import React, { useState } from 'react';
import clsx from 'clsx';

const Tooltip = ({ children, content, disabled = false, className }) => {
  const [isVisible, setIsVisible] = useState(false);

  if (disabled) {
    return children;
  }

  return (
    <div
      className="relative flex-grow h-full"
      onMouseEnter={() => setIsVisible(true)}
      onMouseLeave={() => setIsVisible(false)}
    >
      {children}
      {isVisible && (
        <div
          className={clsx(
            'absolute z-50 px-3 py-2 text-sm font-medium text-white bg-gray-900 rounded-lg shadow-lg',
            'bottom-full left-1/2 transform -translate-x-1/2 mb-2',
            'transition-opacity duration-200 opacity-100',
            'whitespace-nowrap max-w-xs',
            className
          )}
        >
          {content}
          {/* Arrow */}
          <div className="absolute top-full left-1/2 transform -translate-x-1/2 w-0 h-0 border-l-4 border-r-4 border-t-4 border-transparent border-t-gray-900" />
        </div>
      )}
    </div>
  );
};

export default Tooltip;
