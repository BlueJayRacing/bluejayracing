import React from 'react';

interface CustomButtonProps {
  onClick: () => void;
  label: string;
}

const CustomButton: React.FC<CustomButtonProps> = ({ onClick, label }) => {
  return (
    <button
      className="bg-blue-500 text-white px-4 py-2 rounded"
      onClick={onClick}
    >
      {label}
    </button>
  );
};

export default CustomButton;
