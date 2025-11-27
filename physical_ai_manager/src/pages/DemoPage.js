// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Kiwoong Park

import React, { useState, useMemo } from 'react';
import PropTypes from 'prop-types';
import {
  MdArrowBack,
  MdKitchen,
  MdAttachFile,
  MdUsb,
  MdShoppingCart,
  MdClose,
} from 'react-icons/md';
import { GiToothbrush } from 'react-icons/gi';
import { LiaBrushSolid } from 'react-icons/lia';
import { PiScrewdriverDuotone } from 'react-icons/pi';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import toast from 'react-hot-toast';
import clsx from 'clsx';

const PRODUCT_CATALOG = [
  {
    id: 'toothbrush',
    name: 'Toothbrush',
    description: 'Soft bristles for gentle daily care.',
    price: 3.5,
    icon: GiToothbrush,
  },
  {
    id: 'brush',
    name: 'Brush',
    description: 'Multi-purpose handheld cleaning brush.',
    price: 4.0,
    icon: LiaBrushSolid,
  },
  {
    id: 'silicone',
    name: 'Silicone Sealant',
    description: 'Sealant for kitchen and bathroom countertops.',
    price: 6.0,
    icon: MdKitchen,
  },
  {
    id: 'screwdriver',
    name: 'Screwdriver',
    description: 'Compact flat/Phillips combo tool.',
    price: 4.2,
    icon: PiScrewdriverDuotone,
  },
  {
    id: 'clip_set',
    name: 'Clip Set',
    description: 'Metal binder clips for handy storage.',
    price: 2.0,
    icon: MdAttachFile,
  },
  {
    id: 'usb_cable',
    name: 'USB Cable',
    description: '1m USB-A to USB-C quick charge cable.',
    price: 5.5,
    icon: MdUsb,
  },
];

function DemoPage({ onBackToHome }) {
  const { sendDemoCommand } = useRosServiceCaller();
  const [selectedProductIds, setSelectedProductIds] = useState([]);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const selectedProducts = useMemo(() => {
    return PRODUCT_CATALOG.filter((product) => selectedProductIds.includes(product.id));
  }, [selectedProductIds]);

  const handleBackClick = () => {
    onBackToHome();
  };

  const handleToggleProduct = (productId) => {
    setSelectedProductIds((prev) => {
      if (prev.includes(productId)) {
        return prev.filter((id) => id !== productId);
      }
      // TEMPORARY: Allow only one item selection
      return [productId];
      // return [...prev, productId]; // Uncomment for multiple selection
    });
  };

  const handleRemoveSelectedProduct = (productId) => {
    setSelectedProductIds((prev) => prev.filter((id) => id !== productId));
  };

  const handleStartDemo = async () => {
    if (selectedProducts.length === 0) {
      toast.error('Please select at least one product.');
      return;
    }

    try {
      setIsSubmitting(true);
      const orderList = selectedProducts.map((product) => product.name);
      await sendDemoCommand('start', orderList);
      toast.success('Demo command sent successfully.');
    } catch (error) {
      toast.error(`Failed to send demo command: ${error.message}`);
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className="flex flex-col flex-1 h-full bg-gray-50">
      <section className="relative flex-1 w-full px-6 sm:px-8 py-6 flex flex-col lg:flex-row gap-6 overflow-x-hidden">
        <div className="flex-[2] min-w-0 flex flex-col gap-5">
          <div className="space-y-2">
            <p className="text-3xl font-semibold">Select Products</p>
            <p className="text-gray-600 leading-relaxed">
              Tap the shelf cards to add them to the order. Selected products appear on the right.
            </p>
          </div>
          <div className="grid grid-cols-3 grid-rows-2 gap-5 w-full min-w-0 max-w-6xl mx-auto">
            {PRODUCT_CATALOG.map((product) => {
              const isSelected = selectedProductIds.includes(product.id);
              const IconComponent = product.icon;
              return (
                <div key={product.id} className="w-full">
                  <div className="relative w-full" style={{ paddingBottom: '110%' }}>
                    <button
                      type="button"
                      onClick={() => handleToggleProduct(product.id)}
                      className={clsx(
                        'absolute inset-0 flex flex-col items-start gap-3 rounded-3xl px-5 py-6 text-left border shadow-sm transition-all overflow-hidden',
                        isSelected
                          ? 'bg-blue-50 border-blue-400 shadow-lg'
                          : 'bg-white border-gray-200 hover:border-gray-300'
                      )}
                    >
                      <div
                        className="flex items-center justify-center w-full rounded-2xl bg-gray-100 mb-2 flex-shrink-0"
                        style={{ height: '50%' }}
                      >
                        <IconComponent
                          size={60}
                          className="text-gray-600"
                          style={{ maxWidth: '80%', maxHeight: '80%' }}
                        />
                      </div>
                      <div className="flex flex-col flex-1 w-full gap-2 overflow-hidden">
                        <div className="flex w-full justify-between items-start gap-2 flex-shrink-0">
                          <p className="text-xl md:text-2xl font-semibold leading-tight line-clamp-2 flex-1 min-w-0">
                            {product.name}
                          </p>
                          <span
                            className={clsx(
                              'text-xs font-semibold px-2 py-1 rounded-full whitespace-nowrap flex-shrink-0',
                              isSelected ? 'bg-blue-500 text-white' : 'bg-gray-100 text-gray-700'
                            )}
                          >
                            {isSelected ? 'Selected' : 'Tap to Add'}
                          </span>
                        </div>
                        <p
                          className="text-gray-500 text-sm leading-snug overflow-hidden text-ellipsis flex-shrink-0"
                          style={{
                            display: '-webkit-box',
                            WebkitLineClamp: 2,
                            WebkitBoxOrient: 'vertical',
                          }}
                        >
                          {product.description}
                        </p>
                        <p className="text-lg md:text-xl font-bold mt-auto flex-shrink-0">
                          ${product.price.toFixed(2)}
                        </p>
                      </div>
                    </button>
                  </div>
                </div>
              );
            })}
          </div>
        </div>
        <div className="flex-1 bg-white rounded-2xl shadow-lg p-6 flex flex-col max-h-[600px]">
          <div className="flex items-center justify-between pb-4 border-b border-gray-200">
            <div className="flex items-center gap-2">
              <MdShoppingCart size={24} className="text-gray-700" />
              <p className="text-2xl font-bold">Cart</p>
            </div>
            <span className="bg-blue-600 text-white text-sm font-semibold px-3 py-1 rounded-full">
              {selectedProducts.length}
            </span>
          </div>
          {selectedProducts.length === 0 ? (
            <div className="flex-1 flex flex-col items-center justify-center text-center py-8">
              <MdShoppingCart size={64} className="text-gray-300 mb-4" />
              <p className="text-gray-500 text-lg">Your cart is empty</p>
              <p className="text-gray-400 text-sm mt-2">
                Select products from the shelf to add them
              </p>
            </div>
          ) : (
            <>
              <div className="flex-1 overflow-y-auto py-4 space-y-3 scrollbar-thin">
                {selectedProducts.map((product) => {
                  const IconComponent = product.icon;
                  return (
                    <div
                      key={`selected-${product.id}`}
                      className="flex items-center gap-3 p-3 bg-gray-50 rounded-xl hover:bg-gray-100 transition-colors"
                    >
                      <div className="flex items-center justify-center w-14 h-14 rounded-lg bg-white shadow-sm flex-shrink-0">
                        <IconComponent size={32} className="text-gray-600" />
                      </div>
                      <div className="flex-1 min-w-0">
                        <p className="font-semibold text-gray-800 truncate">{product.name}</p>
                        <p className="text-lg font-bold text-blue-600">
                          ${product.price.toFixed(2)}
                        </p>
                      </div>
                      <button
                        type="button"
                        className="text-red-500 hover:text-red-600 hover:bg-red-50 p-2 rounded-lg transition-colors flex-shrink-0"
                        onClick={() => handleRemoveSelectedProduct(product.id)}
                        title="Remove from cart"
                      >
                        <MdClose size={20} />
                      </button>
                    </div>
                  );
                })}
              </div>
              <div className="pt-4 border-t border-gray-200 space-y-3">
                <div className="flex justify-between items-center">
                  <p className="text-lg font-semibold text-gray-700">Total</p>
                  <p className="text-2xl font-bold text-gray-900">
                    ${selectedProducts.reduce((sum, p) => sum + p.price, 0).toFixed(2)}
                  </p>
                </div>
                <button
                  type="button"
                  className="w-full px-6 py-3 rounded-xl bg-blue-600 text-white font-semibold shadow-lg
                    hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
                  onClick={handleStartDemo}
                  disabled={isSubmitting || selectedProducts.length === 0}
                >
                  {isSubmitting ? 'Processing...' : 'Place Order'}
                </button>
              </div>
            </>
          )}
        </div>
        <button
          type="button"
          onClick={handleBackClick}
          className="absolute bottom-4 right-4 text-xs px-3 py-1 rounded-full border border-gray-300
            shadow-sm hover:bg-gray-100 transition-colors flex items-center gap-1"
        >
          <MdArrowBack size={14} />
          <span>Back</span>
        </button>
      </section>
    </div>
  );
}

DemoPage.propTypes = {
  onBackToHome: PropTypes.func,
};

DemoPage.defaultProps = {
  onBackToHome: () => {},
};

export default DemoPage;
