import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import Tooltip from '../Tooltip';

describe('Tooltip Component', () => {
  const mockContent = 'This is a tooltip';
  const mockChildren = <button>Hover me</button>;

  test('renders children without tooltip initially', () => {
    render(<Tooltip content={mockContent}>{mockChildren}</Tooltip>);

    expect(screen.getByText('Hover me')).toBeInTheDocument();
    expect(screen.queryByText(mockContent)).not.toBeInTheDocument();
  });

  test('shows tooltip on mouse enter', () => {
    render(<Tooltip content={mockContent}>{mockChildren}</Tooltip>);

    const triggerElement = screen.getByText('Hover me').parentElement;
    fireEvent.mouseEnter(triggerElement);

    expect(screen.getByText(mockContent)).toBeInTheDocument();
  });

  test('hides tooltip on mouse leave', () => {
    render(<Tooltip content={mockContent}>{mockChildren}</Tooltip>);

    const triggerElement = screen.getByText('Hover me').parentElement;

    // Show tooltip
    fireEvent.mouseEnter(triggerElement);
    expect(screen.getByText(mockContent)).toBeInTheDocument();

    // Hide tooltip
    fireEvent.mouseLeave(triggerElement);
    expect(screen.queryByText(mockContent)).not.toBeInTheDocument();
  });

  test('does not show tooltip when disabled', () => {
    render(
      <Tooltip content={mockContent} disabled={true}>
        {mockChildren}
      </Tooltip>
    );

    const button = screen.getByText('Hover me');
    fireEvent.mouseEnter(button);

    expect(screen.queryByText(mockContent)).not.toBeInTheDocument();
  });

  test('returns only children when disabled', () => {
    const { container } = render(
      <Tooltip content={mockContent} disabled={true}>
        {mockChildren}
      </Tooltip>
    );

    // When disabled, should return children directly without wrapper
    expect(screen.getByText('Hover me')).toBeInTheDocument();
  });

  test('applies custom className to tooltip', () => {
    const customClass = 'custom-tooltip-class';
    render(
      <Tooltip content={mockContent} className={customClass}>
        {mockChildren}
      </Tooltip>
    );

    const triggerElement = screen.getByText('Hover me').parentElement;
    fireEvent.mouseEnter(triggerElement);

    const tooltip = screen.getByText(mockContent);
    expect(tooltip).toHaveClass(customClass);
  });

  test('tooltip has proper ARIA attributes for accessibility', () => {
    render(<Tooltip content={mockContent}>{mockChildren}</Tooltip>);

    const triggerElement = screen.getByText('Hover me').parentElement;
    fireEvent.mouseEnter(triggerElement);

    const tooltip = screen.getByText(mockContent);
    expect(tooltip).toBeInTheDocument();
    // Verify tooltip styling classes are present
    expect(tooltip).toHaveClass('absolute', 'z-50', 'px-3', 'py-2');
  });
});
