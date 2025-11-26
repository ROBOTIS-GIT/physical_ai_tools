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

import React from 'react';
import PropTypes from 'prop-types';
import { MdArrowBack } from 'react-icons/md';

function DemoPage({ onBackToHome }) {
  const handleBackClick = () => {
    onBackToHome();
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
      <section className="flex-1 flex items-center justify-center p-8">
        <div className="max-w-3xl text-center space-y-4">
          <p className="text-3xl font-semibold">Welcome to the Demo Page</p>
          <p className="text-gray-600 leading-relaxed">
            Use this space to highlight guided walkthroughs, marketing content, or live showcases.
            The left navigation is intentionally hidden to provide an immersive view. Select the
            Back button whenever you are ready to return to the Home page.
          </p>
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
