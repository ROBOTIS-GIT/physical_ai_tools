import React from 'react';
import { render, screen } from '@testing-library/react';
import ProgressBar from '../ProgressBar';

describe('ProgressBar Component', () => {
  test('renders with default percent value', () => {
    render(<ProgressBar />);

    const percentageText = screen.getByText('0%');
    expect(percentageText).toBeInTheDocument();
  });

  test('renders with custom percent value', () => {
    render(<ProgressBar percent={75} />);

    const percentageText = screen.getByText('75%');
    expect(percentageText).toBeInTheDocument();
  });

  test('applies correct width style based on percent', () => {
    const { container } = render(<ProgressBar percent={50} />);

    const progressFill = container.querySelector('.bg-gray-700');
    expect(progressFill).toHaveStyle('width: 50%');
  });

  test('applies white text color when percent > 50', () => {
    render(<ProgressBar percent={75} />);

    const percentageText = screen.getByText('75%');
    expect(percentageText).toHaveClass('text-white');
  });

  test('applies dark text color when percent <= 50', () => {
    render(<ProgressBar percent={25} />);

    const percentageText = screen.getByText('25%');
    expect(percentageText).toHaveClass('text-gray-800');
  });

  test('handles edge case of 100%', () => {
    const { container } = render(<ProgressBar percent={100} />);

    const percentageText = screen.getByText('100%');
    const progressFill = container.querySelector('.bg-gray-700');

    expect(percentageText).toBeInTheDocument();
    expect(progressFill).toHaveStyle('width: 100%');
  });

  test('handles edge case of 0%', () => {
    const { container } = render(<ProgressBar percent={0} />);

    const percentageText = screen.getByText('0%');
    const progressFill = container.querySelector('.bg-gray-700');

    expect(percentageText).toBeInTheDocument();
    expect(progressFill).toHaveStyle('width: 0%');
  });
});
