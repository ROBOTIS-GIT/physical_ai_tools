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
import { MdArrowBack, MdShoppingCart, MdClose } from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import toast from 'react-hot-toast';
import clsx from 'clsx';

const PRODUCT_CATALOG = [
  {
    id: 'tube',
    name: 'Tube',
    description: 'Tube for wrapping and sealing.',
    price: 3.5,
    image: '/image/tube.png',
  },
  {
    id: 'scissors',
    name: 'Scissors',
    description: 'Scissors for cutting and trimming.',
    price: 3.5,
    image: '/image/scissors.png',
  },
  {
    id: 'adjustable wrench',
    name: 'Wrench',
    description: 'Adjustable wrench for tightening and loosening bolts.',
    price: 2.0,
    image: '/image/wrench.png',
  },
  {
    id: 'pliers',
    name: 'Pliers',
    description: 'Pliers for gripping and cutting.',
    price: 6.0,
    image: '/image/pliers.png',
  },
  {
    id: 'paintbrush',
    name: 'Paintbrush',
    description: 'Paintbrush for painting and wall covering.',
    price: 4.0,
    image: '/image/paintbrush.png',
  },
  {
    id: 'screwdriver',
    name: 'Screwdriver',
    description: 'Compact flat/Phillips combo tool.',
    price: 4.2,
    image: '/image/screwdriver.png',
  },
  {
    id: 'roller',
    name: 'Roller',
    description: 'Roller for painting and wall covering.',
    price: 5.5,
    image: '/image/roller.png',
  },
  {
    id: 'shovel',
    name: 'Shovel',
    description: 'Shovel for digging and planting.',
    price: 5.5,
    image: '/image/shovel.png',
  },
];

function DemoPage({ onBackToHome }) {
  const { sendDemoCommand } = useRosServiceCaller();
  const [selectedProductIds, setSelectedProductIds] = useState([]);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const selectedProducts = useMemo(() => {
    return PRODUCT_CATALOG.filter((product) => selectedProductIds.includes(product.id));
  }, [selectedProductIds]);

  // Style classes
  const classPageContainer = clsx('flex', 'flex-col', 'flex-1', 'h-full', 'bg-gray-50');

  const classMainSection = clsx(
    'relative',
    'flex-1',
    'w-full',
    'px-6',
    'sm:px-8',
    'py-6',
    'flex',
    'flex-col',
    'lg:flex-row',
    'gap-6',
    'overflow-x-hidden'
  );

  const classProductGridContainer = clsx('flex-[2]', 'min-w-0', 'flex', 'flex-col', 'gap-5');

  const classProductGrid = clsx(
    'grid',
    'grid-cols-4',
    'grid-rows-2',
    'gap-4',
    'w-full',
    'min-w-0',
    'max-w-8xl',
    'mx-auto'
  );

  const classCartContainer = clsx(
    'flex-1',
    'bg-white',
    'rounded-2xl',
    'shadow-lg',
    'p-6',
    'flex',
    'flex-col',
    'max-h-[600px]'
  );

  const classCartHeader = clsx(
    'flex',
    'items-center',
    'justify-between',
    'pb-4',
    'border-b',
    'border-gray-200'
  );

  const classCartEmpty = clsx(
    'flex-1',
    'flex',
    'flex-col',
    'items-center',
    'justify-center',
    'text-center',
    'py-8'
  );

  const classCartList = clsx('flex-1', 'overflow-y-auto', 'py-4', 'space-y-3', 'scrollbar-thin');

  const classCartFooter = clsx('pt-4', 'border-t', 'border-gray-200', 'space-y-3');

  const classBackButton = clsx(
    'absolute',
    'bottom-4',
    'right-4',
    'text-xs',
    'px-3',
    'py-1',
    'rounded-full',
    'border',
    'border-gray-300',
    'shadow-sm',
    'hover:bg-gray-100',
    'transition-colors',
    'flex',
    'items-center',
    'gap-1'
  );

  const getProductCardClass = (isSelected) =>
    clsx(
      'absolute',
      'inset-0',
      'flex',
      'flex-col',
      'items-start',
      'rounded-2xl',
      'text-left',
      'border',
      'shadow-sm',
      'transition-all',
      'overflow-hidden',
      {
        'bg-blue-50': isSelected,
        'border-blue-400': isSelected,
        'shadow-lg': isSelected,
        'bg-white': !isSelected,
        'border-gray-200': !isSelected,
        'hover:border-gray-300': !isSelected,
      }
    );

  const classProductIconContainer = clsx(
    'flex',
    'items-center',
    'justify-center',
    'w-full',
    'rounded-xl',
    'bg-gray-100',
    'mb-1',
    'flex-shrink-0'
  );

  const classProductDetails = clsx(
    'flex',
    'flex-col',
    'flex-1',
    'w-full',
    'overflow-hidden',
    'min-h-0'
  );

  const classProductHeader = clsx(
    'flex',
    'w-full',
    'justify-between',
    'items-start',
    'flex-shrink-0',
    'min-w-0'
  );

  const classProductName = clsx(
    'font-semibold',
    'leading-tight',
    'line-clamp-2',
    'flex-1',
    'min-w-0',
    'overflow-hidden',
    'break-words'
  );

  const getProductBadgeClass = (isSelected) =>
    clsx(
      'font-semibold',
      'px-1.5',
      'py-0.5',
      'rounded-full',
      'whitespace-nowrap',
      'flex-shrink-0',
      {
        'bg-blue-500': isSelected,
        'text-white': isSelected,
        'bg-gray-100': !isSelected,
        'text-gray-700': !isSelected,
      }
    );

  const classProductDescription = clsx(
    'text-gray-500',
    'leading-snug',
    'overflow-hidden',
    'text-ellipsis',
    'flex-shrink-0',
    'w-full'
  );

  const classProductPrice = clsx('font-bold', 'mt-auto', 'flex-shrink-0', 'w-full', 'truncate');

  const classCartItem = clsx(
    'flex',
    'items-center',
    'gap-3',
    'p-3',
    'bg-gray-50',
    'rounded-xl',
    'hover:bg-gray-100',
    'transition-colors'
  );

  const classCartItemIcon = clsx(
    'flex',
    'items-center',
    'justify-center',
    'w-14',
    'h-14',
    'rounded-lg',
    'bg-white',
    'shadow-sm',
    'flex-shrink-0'
  );

  const classCartItemDetails = clsx('flex-1', 'min-w-0');

  const classCartItemRemove = clsx(
    'text-red-500',
    'hover:text-red-600',
    'hover:bg-red-50',
    'p-2',
    'rounded-lg',
    'transition-colors',
    'flex-shrink-0'
  );

  const classPlaceOrderButton = clsx(
    'w-full',
    'px-6',
    'py-3',
    'rounded-xl',
    'bg-blue-600',
    'text-white',
    'font-semibold',
    'shadow-lg',
    'hover:bg-blue-700',
    'disabled:opacity-50',
    'disabled:cursor-not-allowed',
    'transition-colors'
  );

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
    <div className={classPageContainer}>
      <section className={classMainSection}>
        <div className={classProductGridContainer}>
          <div className="space-y-2">
            <p className="text-3xl font-semibold">Select Products</p>
            <p className="text-gray-600 leading-relaxed">
              Tap the shelf cards to add them to the order. Selected products appear on the right.
            </p>
          </div>
          <div className={classProductGrid}>
            {PRODUCT_CATALOG.map((product) => {
              const isSelected = selectedProductIds.includes(product.id);
              return (
                <div key={product.id} className="w-full">
                  <div className="relative w-full" style={{ paddingBottom: '130%' }}>
                    <button
                      type="button"
                      onClick={() => handleToggleProduct(product.id)}
                      className={getProductCardClass(isSelected)}
                      style={{
                        padding: 'clamp(0.375rem, 1.5vw, 1rem)',
                        gap: 'clamp(0.125rem, 0.8vw, 0.5rem)',
                      }}
                    >
                      <div
                        className={classProductIconContainer}
                        style={{
                          height: '58%',
                          minHeight: '0',
                          flexShrink: 0,
                        }}
                      >
                        <img
                          src={product.image}
                          alt={product.name}
                          className="w-full h-full object-cover rounded-xl"
                        />
                      </div>
                      <div
                        className={classProductDetails}
                        style={{
                          gap: 'clamp(0.0625rem, 0.5vw, 0.5rem)',
                        }}
                      >
                        <div
                          className={classProductHeader}
                          style={{
                            gap: 'clamp(0.125rem, 0.6vw, 0.75rem)',
                          }}
                        >
                          <p
                            className={classProductName}
                            style={{
                              fontSize: 'clamp(0.625rem, 1.2vw, 1.5rem)',
                            }}
                          >
                            {product.name}
                          </p>
                          <span
                            className={getProductBadgeClass(isSelected)}
                            style={{
                              fontSize: 'clamp(0.4rem, 0.75vw, 0.875rem)',
                              padding:
                                'clamp(0.0625rem, 0.3vw, 0.375rem) clamp(0.125rem, 0.5vw, 0.5rem)',
                            }}
                          >
                            {isSelected ? 'Selected' : 'Add'}
                          </span>
                        </div>
                        <p
                          className={classProductDescription}
                          style={{
                            display: '-webkit-box',
                            WebkitLineClamp: 2,
                            WebkitBoxOrient: 'vertical',
                            fontSize: 'clamp(0.4rem, 0.85vw, 1rem)',
                            lineHeight: 'clamp(0.6rem, 1.1vw, 1.4rem)',
                          }}
                        >
                          {product.description}
                        </p>
                        <p
                          className={classProductPrice}
                          style={{
                            fontSize: 'clamp(0.5rem, 1.1vw, 1.375rem)',
                          }}
                        >
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
        <div className={classCartContainer}>
          <div className={classCartHeader}>
            <div className="flex items-center gap-2">
              <MdShoppingCart size={24} className="text-gray-700" />
              <p className="text-2xl font-bold">Cart</p>
            </div>
            <span className="bg-blue-600 text-white text-sm font-semibold px-3 py-1 rounded-full">
              {selectedProducts.length}
            </span>
          </div>
          {selectedProducts.length === 0 ? (
            <div className={classCartEmpty}>
              <MdShoppingCart size={64} className="text-gray-300 mb-4" />
              <p className="text-gray-500 text-lg">Your cart is empty</p>
              <p className="text-gray-400 text-sm mt-2">
                Select products from the shelf to add them
              </p>
            </div>
          ) : (
            <>
              <div className={classCartList}>
                {selectedProducts.map((product) => {
                  return (
                    <div key={`selected-${product.id}`} className={classCartItem}>
                      <div className={classCartItemIcon}>
                        <img
                          src={product.image}
                          alt={product.name}
                          className="w-full h-full object-cover rounded-lg"
                        />
                      </div>
                      <div className={classCartItemDetails}>
                        <p className="font-semibold text-gray-800 truncate">{product.name}</p>
                        <p className="text-lg font-bold text-blue-600">
                          ${product.price.toFixed(2)}
                        </p>
                      </div>
                      <button
                        type="button"
                        className={classCartItemRemove}
                        onClick={() => handleRemoveSelectedProduct(product.id)}
                        title="Remove from cart"
                      >
                        <MdClose size={20} />
                      </button>
                    </div>
                  );
                })}
              </div>
              <div className={classCartFooter}>
                <div className="flex justify-between items-center">
                  <p className="text-lg font-semibold text-gray-700">Total</p>
                  <p className="text-2xl font-bold text-gray-900">
                    ${selectedProducts.reduce((sum, p) => sum + p.price, 0).toFixed(2)}
                  </p>
                </div>
                <button
                  type="button"
                  className={classPlaceOrderButton}
                  onClick={handleStartDemo}
                  disabled={isSubmitting || selectedProducts.length === 0}
                >
                  {isSubmitting ? 'Processing...' : 'Place Order'}
                </button>
              </div>
            </>
          )}
        </div>
        <button type="button" onClick={handleBackClick} className={classBackButton}>
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
