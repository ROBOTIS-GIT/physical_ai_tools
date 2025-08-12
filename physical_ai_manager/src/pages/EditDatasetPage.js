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

import React, { useEffect } from 'react';
import toast, { useToasterStore } from 'react-hot-toast';

const DatasetListInput = () => {
  return <div>DatasetListInput</div>;
};

export default function EditDatasetPage() {
  // Toast limit implementation using useToasterStore
  const { toasts } = useToasterStore();
  const TOAST_LIMIT = 3;

  useEffect(() => {
    toasts
      .filter((t) => t.visible) // Only consider visible toasts
      .filter((_, i) => i >= TOAST_LIMIT) // Is toast index over limit?
      .forEach((t) => toast.dismiss(t.id)); // Dismiss – Use toast.remove(t.id) for no exit animation
  }, [toasts]);

  return (
    <div className="w-full h-full flex flex-col items-start justify-start pt-10">
      <div className="w-full h-full flex flex-col items-start justify-start pt-10">
        <div className="w-full h-full flex flex-col items-start justify-start p-10 gap-8">
          <div className="w-full h-full flex flex-col items-start justify-start">
            <h1 className="text-2xl font-bold">Edit Dataset</h1>
            <DatasetListInput />
          </div>
        </div>
      </div>
    </div>
  );
}
