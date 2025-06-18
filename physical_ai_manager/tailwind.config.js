/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ['./src/**/*.{js,jsx,ts,tsx}'],
  theme: {
    extend: {
      spacing: {
        4: '1rem',
        8: '2rem',
        25: '6.25rem',
        30: '7.5rem',
        35: '8.75rem',
        38: '9.5rem',
        65: '16.25rem',
        75: '18.75rem',
        80: '20rem',
        100: '25rem',
      },
      maxHeight: {
        25: '6.25rem',
        30: '7.5rem',
        50: '12.5rem',
        75: '18.75rem',
        100: '25rem',
      },
      minHeight: {
        4: '1rem',
        8: '2rem',
      },
      minWidth: {
        20: '5rem',
        30: '7.5rem',
        65: '16.25rem',
      },
      width: {
        35: '8.75rem',
        80: '20rem',
      },
      flex: {
        6: '6 6 0%',
      },
    },
  },
  plugins: [],
};
