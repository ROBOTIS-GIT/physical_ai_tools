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

import React, { useState, useRef, useLayoutEffect } from 'react';
import { createPortal } from 'react-dom';
import clsx from 'clsx';

const Tooltip = ({ children, content, disabled = false, className, position = 'top' }) => {
  const [isVisible, setIsVisible] = useState(false);
  const [coords, setCoords] = useState({ left: 0, top: 0 });
  const anchorRef = useRef(null);

  const isBottom = position === 'bottom';

  // Reposition on show to follow the anchor element. Rendered via portal
  // into body so that ancestor overflow:auto/hidden does not clip it.
  useLayoutEffect(() => {
    if (!isVisible || !anchorRef.current) return;
    const rect = anchorRef.current.getBoundingClientRect();
    setCoords({
      left: rect.left + rect.width / 2,
      top: isBottom ? rect.bottom + 8 : rect.top - 8,
    });
  }, [isVisible, isBottom]);

  if (disabled) {
    return <div className={className}>{children}</div>;
  }

  return (
    <>
      <div
        ref={anchorRef}
        className={clsx('inline-flex', className)}
        onMouseEnter={() => setIsVisible(true)}
        onMouseLeave={() => setIsVisible(false)}
      >
        {children}
      </div>
      {isVisible && createPortal(
        <div
          className={clsx(
            'fixed z-[9999] px-3 py-2 text-sm font-medium text-white bg-gray-900 rounded-lg shadow-lg max-w-xs break-words',
            isBottom ? 'transform -translate-x-1/2' : 'transform -translate-x-1/2 -translate-y-full'
          )}
          style={{ left: coords.left, top: coords.top, pointerEvents: 'none' }}
        >
          {content}
          <div
            className={clsx(
              'absolute left-1/2 transform -translate-x-1/2 w-0 h-0 border-l-4 border-r-4 border-transparent',
              isBottom
                ? 'bottom-full border-b-4 border-b-gray-900'
                : 'top-full border-t-4 border-t-gray-900'
            )}
          />
        </div>,
        document.body
      )}
    </>
  );
};

export default Tooltip;
