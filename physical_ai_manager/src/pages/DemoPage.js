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
import { MdArrowBack } from 'react-icons/md';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import toast from 'react-hot-toast';

function DemoPage({ onBackToHome }) {
  const { sendDemoCommand } = useRosServiceCaller();
  const [instructionItems, setInstructionItems] = useState(['']);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const sanitizedItems = useMemo(() => {
    return instructionItems.map((item) => item.trim()).filter((item) => item !== '');
  }, [instructionItems]);

  const handleBackClick = () => {
    onBackToHome();
  };

  const handleInstructionChange = (index, value) => {
    setInstructionItems((prev) => {
      const nextItems = [...prev];
      nextItems[index] = value;
      return nextItems;
    });
  };

  const handleAddInstruction = () => {
    setInstructionItems((prev) => [...prev, '']);
  };

  const handleRemoveInstruction = (index) => {
    setInstructionItems((prev) => {
      if (prev.length === 1) {
        return prev;
      }
      return prev.filter((_, idx) => idx !== index);
    });
  };

  const handleStartDemo = async () => {
    if (sanitizedItems.length === 0) {
      toast.error('Please enter at least one instruction.');
      return;
    }

    try {
      setIsSubmitting(true);
      await sendDemoCommand('start', sanitizedItems);
      toast.success('Demo command sent successfully.');
    } catch (error) {
      toast.error(`Failed to send demo command: ${error.message}`);
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className="flex flex-col flex-1 h-full bg-gray-50">
      <header className="flex items-center justify-between px-6 py-4 border-b border-gray-200">
        <button
          type="button"
          onClick={handleBackClick}
          className="flex items-center gap-2 px-4 py-2 rounded-xl border border-gray-300
            shadow-sm hover:bg-gray-100 transition-colors"
        >
          <MdArrowBack size={20} />
          <span className="font-semibold text-sm">Back</span>
        </button>
        <div className="text-center">
          <p className="text-xl font-bold">Demo Mode</p>
          <p className="text-sm text-gray-500">
            Showcase the full-screen experience without navigation chrome
          </p>
        </div>
        <div className="w-24" />
      </header>
      <section className="flex-1 flex flex-col gap-8 items-center justify-center p-8">
        <div className="w-full max-w-4xl space-y-4 text-center">
          <p className="text-3xl font-semibold">Configure Demo Instructions</p>
          <p className="text-gray-600 leading-relaxed">
            Provide multiple step-by-step instructions. They will be forwarded to the demo service
            as an ordered list right after you press the Start button.
          </p>
        </div>
        <div className="w-full max-w-4xl bg-white rounded-2xl shadow-md p-6 space-y-4">
          {instructionItems.map((item, index) => (
            <div key={`demo-item-${index}`} className="flex gap-3 items-start">
              <span className="text-gray-500 pt-2">{index + 1}.</span>
              <textarea
                className="flex-1 border border-gray-300 rounded-xl px-3 py-2 min-h-[64px]
                  focus:outline-none focus:ring-2 focus:ring-blue-400"
                placeholder="Enter instruction text"
                value={item}
                onChange={(event) => handleInstructionChange(index, event.target.value)}
              />
              <button
                type="button"
                className="px-3 py-2 text-sm text-red-500 border border-red-200 rounded-xl
                  hover:bg-red-50 disabled:opacity-40 disabled:cursor-not-allowed"
                onClick={() => handleRemoveInstruction(index)}
                disabled={instructionItems.length === 1}
              >
                Remove
              </button>
            </div>
          ))}
          <div className="flex justify-between">
            <button
              type="button"
              className="px-4 py-2 rounded-xl border border-gray-300 text-sm font-semibold
                hover:bg-gray-100"
              onClick={handleAddInstruction}
            >
              Add Instruction
            </button>
            <button
              type="button"
              className="px-6 py-2 rounded-xl bg-blue-600 text-white font-semibold shadow
                hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed"
              onClick={handleStartDemo}
              disabled={isSubmitting}
            >
              {isSubmitting ? 'Starting...' : 'Start'}
            </button>
          </div>
        </div>
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
