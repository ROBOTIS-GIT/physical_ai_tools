import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import TagInput from '../TagInput';

describe('TagInput Component', () => {
  const mockOnChange = jest.fn();

  beforeEach(() => {
    mockOnChange.mockClear();
  });

  test('renders with empty tags', () => {
    render(<TagInput tags={[]} onChange={mockOnChange} />);

    const input = screen.getByPlaceholderText('Add tags');
    expect(input).toBeInTheDocument();
  });

  test('renders existing tags', () => {
    const tags = ['tag1', 'tag2', 'tag3'];
    render(<TagInput tags={tags} onChange={mockOnChange} />);

    tags.forEach((tag) => {
      expect(screen.getByText(tag)).toBeInTheDocument();
    });
  });

  test('adds tag on Enter key press', async () => {
    const user = userEvent.setup();
    render(<TagInput tags={[]} onChange={mockOnChange} />);

    const input = screen.getByPlaceholderText('Add tags');
    await user.type(input, 'new tag');
    await user.keyboard('{Enter}');

    expect(mockOnChange).toHaveBeenCalledWith(['new tag']);
  });

  test('adds tag on comma input', async () => {
    const user = userEvent.setup();
    render(<TagInput tags={[]} onChange={mockOnChange} />);

    const input = screen.getByPlaceholderText('Add tags');
    await user.type(input, 'tag1,tag2,');

    expect(mockOnChange).toHaveBeenCalledWith(['tag1', 'tag2']);
  });

  test('removes tag when click remove button', async () => {
    const user = userEvent.setup();
    const tags = ['tag1', 'tag2'];
    render(<TagInput tags={tags} onChange={mockOnChange} />);

    const removeButtons = screen.getAllByText('×');
    await user.click(removeButtons[0]);

    expect(mockOnChange).toHaveBeenCalledWith(['tag2']);
  });

  test('removes last tag on backspace when input is empty', async () => {
    const user = userEvent.setup();
    const tags = ['tag1', 'tag2'];
    render(<TagInput tags={tags} onChange={mockOnChange} />);

    const input = screen.getByRole('textbox');
    await user.click(input);
    await user.keyboard('{Backspace}');

    expect(mockOnChange).toHaveBeenCalledWith(['tag1']);
  });

  test('does not add duplicate tags', async () => {
    const user = userEvent.setup();
    const tags = ['existing-tag'];
    render(<TagInput tags={tags} onChange={mockOnChange} />);

    const input = screen.getByRole('textbox');
    await user.type(input, 'existing-tag');
    await user.keyboard('{Enter}');

    expect(mockOnChange).not.toHaveBeenCalled();
  });

  test('trims whitespace from tags', async () => {
    const user = userEvent.setup();
    render(<TagInput tags={[]} onChange={mockOnChange} />);

    const input = screen.getByPlaceholderText('Add tags');
    await user.type(input, '  spaced tag  ');
    await user.keyboard('{Enter}');

    expect(mockOnChange).toHaveBeenCalledWith(['spaced tag']);
  });

  test('does not add empty tags', async () => {
    const user = userEvent.setup();
    render(<TagInput tags={[]} onChange={mockOnChange} />);

    const input = screen.getByPlaceholderText('Add tags');
    await user.type(input, '   ');
    await user.keyboard('{Enter}');

    expect(mockOnChange).not.toHaveBeenCalled();
  });

  test('is disabled when disabled prop is true', () => {
    const tags = ['tag1'];
    render(<TagInput tags={tags} onChange={mockOnChange} disabled={true} />);

    const input = screen.getByRole('textbox');
    expect(input).toBeDisabled();

    // Remove buttons should not be present when disabled
    expect(screen.queryByText('×')).not.toBeInTheDocument();
  });

  test('applies disabled styling when disabled', () => {
    render(<TagInput tags={[]} onChange={mockOnChange} disabled={true} />);

    const container = screen.getByRole('textbox').closest('div');
    expect(container).toHaveClass('bg-gray-100', 'cursor-not-allowed');
  });

  test('applies custom className', () => {
    const customClass = 'custom-tag-input';
    render(<TagInput tags={[]} onChange={mockOnChange} className={customClass} />);

    const container = screen.getByRole('textbox').closest('div');
    expect(container).toHaveClass(customClass);
  });

  test('clears input after adding tag', async () => {
    const user = userEvent.setup();
    render(<TagInput tags={[]} onChange={mockOnChange} />);

    const input = screen.getByPlaceholderText('Add tags');
    await user.type(input, 'test tag');
    await user.keyboard('{Enter}');

    expect(input).toHaveValue('');
  });

  test('handles multiple comma-separated tags at once', async () => {
    const user = userEvent.setup();
    render(<TagInput tags={[]} onChange={mockOnChange} />);

    const input = screen.getByPlaceholderText('Add tags');
    await user.type(input, 'tag1, tag2 , tag3,');

    expect(mockOnChange).toHaveBeenCalledWith(['tag1', 'tag2', 'tag3']);
  });
});
