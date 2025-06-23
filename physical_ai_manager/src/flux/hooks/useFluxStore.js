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

import { useState, useEffect } from 'react';

/**
 * Custom hook to connect React components to Flux stores
 * @param {Object} store - Flux store instance
 * @param {Function} getStateFromStore - Function to extract state from store
 * @returns {any} State from store
 */
export function useFluxStore(store, getStateFromStore) {
  const [state, setState] = useState(() => getStateFromStore(store));

  useEffect(() => {
    const handleStoreChange = () => {
      setState(getStateFromStore(store));
    };

    store.addChangeListener(handleStoreChange);

    return () => {
      store.removeChangeListener(handleStoreChange);
    };
  }, [store, getStateFromStore]);

  return state;
}

export default useFluxStore;
