import React, { useState, useRef } from 'react';
import axios from 'axios';
import {
  Button,
  LinearProgress,
  TextField,
  Typography,
  Box,
  Alert,
  Snackbar,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
} from '@mui/material';
import CheckCircleIcon from '@mui/icons-material/CheckCircle';
import ErrorIcon from '@mui/icons-material/Error';

export function FileTransfer({ serverUrl }) {
  const [selectedFile, setSelectedFile] = useState(null);
  const [uploadPath, setUploadPath] = useState('');
  const [downloadPath, setDownloadPath] = useState('');
  const [uploadProgress, setUploadProgress] = useState(0);
  const [downloadProgress, setDownloadProgress] = useState(0);
  const [uploadSpeed, setUploadSpeed] = useState(0);
  const [downloadSpeed, setDownloadSpeed] = useState(0);
  const [uploadTimeLeft, setUploadTimeLeft] = useState('');
  const [downloadTimeLeft, setDownloadTimeLeft] = useState('');
  const [error, setError] = useState('');
  const [lastUploadedChunk, setLastUploadedChunk] = useState(0);
  const [isPaused, setIsPaused] = useState(false);
  const [transferId, setTransferId] = useState(null);
  const [uploadComplete, setUploadComplete] = useState(false);
  const [showSuccess, setShowSuccess] = useState(false);
  const [showOverwriteDialog, setShowOverwriteDialog] = useState(false);
  const [pendingUpload, setPendingUpload] = useState(null);
  const abortControllerRef = useRef(null);
  const isUploadingRef = useRef(false);
  const isPausedRef = useRef(false);
  const lastSuccessfulChunkRef = useRef(-1);

  const handleFileSelect = (event) => {
    const file = event.target.files[0];
    if (file) {
      setSelectedFile(file);
      setError('');
      setUploadComplete(false);
      setUploadProgress(0);
      lastSuccessfulChunkRef.current = -1;
    }
  };

  const checkFileExists = async (filePath) => {
    try {
      const response = await axios.head(`${serverUrl}/check-file/${encodeURIComponent(filePath)}`);
      return response.status === 200;
    } catch (error) {
      if (error.response && error.response.status === 404) {
        return false;
      }
      throw error;
    }
  };

  const normalizePath = (path) => {
    // 앞뒤 슬래시 제거
    let normalized = path.replace(/^\/+|\/+$/g, '');

    // /home으로 시작하는 경우 제거
    normalized = normalized.replace(/^home\//, '');

    return normalized;
  };

  const handleUploadClick = async () => {
    console.log('Upload button clicked');
    console.log('Selected file:', selectedFile);
    console.log('Upload path:', uploadPath);

    if (!selectedFile) {
      setError('파일을 선택해주세요.');
      return;
    }

    if (!uploadPath) {
      setError('업로드 경로를 입력해주세요.');
      return;
    }

    try {
      const normalizedPath = normalizePath(uploadPath);
      const fullPath = `/home/${normalizedPath}/${selectedFile.name}`;
      console.log('Normalized full path:', fullPath);

      const exists = await checkFileExists(fullPath);
      console.log('File exists:', exists);

      if (exists) {
        setShowOverwriteDialog(true);
        setPendingUpload({ path: fullPath });
      } else {
        console.log('Starting new upload');
        isUploadingRef.current = true;
        lastSuccessfulChunkRef.current = -1;
        uploadWithRetry();
      }
    } catch (err) {
      console.error('Upload initialization error:', err);
      setError('파일 존재 여부 확인 실패: ' + err.message);
    }
  };

  const handleOverwriteConfirm = () => {
    console.log('Overwrite confirmed');
    setShowOverwriteDialog(false);
    if (pendingUpload) {
      isUploadingRef.current = true;
      lastSuccessfulChunkRef.current = -1;
      uploadWithRetry();
    }
  };

  const handleOverwriteCancel = () => {
    console.log('Overwrite cancelled');
    setShowOverwriteDialog(false);
    setPendingUpload(null);
  };

  const handlePause = async () => {
    if (!transferId || !isUploadingRef.current) return;

    try {
      console.log('Pausing upload...');
      // 먼저 ref 상태를 변경
      isPausedRef.current = true;
      isUploadingRef.current = false;
      // UI 상태도 변경
      setIsPaused(true);

      // AbortController 정리
      if (abortControllerRef.current) {
        abortControllerRef.current.abort();
        abortControllerRef.current = null;
      }

      // 서버에 일시 중지 요청
      await axios.post(`${serverUrl}/pause/${transferId}`);
      console.log('Upload paused successfully');
    } catch (err) {
      console.error('Pause error:', err);
      setError('전송 일시 중지 실패: ' + err.message);
      // 에러 발생 시 상태 복구
      isPausedRef.current = false;
      isUploadingRef.current = true;
      setIsPaused(false);
    }
  };

  const handleResume = async () => {
    if (!transferId) {
      console.log('No transfer ID available');
      return;
    }

    try {
      console.log('Resuming upload...');
      // 현재 전송 상태 확인
      const statusResponse = await axios.get(`${serverUrl}/transfer-status/${transferId}`);
      const { uploaded_bytes, total_bytes } = statusResponse.data;

      console.log('Server status check:', {
        uploaded_bytes,
        total_bytes,
        isComplete: uploaded_bytes >= total_bytes,
        clientLastChunk: lastSuccessfulChunkRef.current,
      });

      // 실제로 업로드가 완료되었는지 확인 (보수적 접근)
      if (
        uploaded_bytes >= total_bytes &&
        lastSuccessfulChunkRef.current >= Math.ceil(total_bytes / (1 * 1024 * 1024)) - 1
      ) {
        console.log('Upload actually completed - both server and client agree');
        setUploadComplete(true);
        setShowSuccess(true);
        return;
      }

      // 서버에 재개 요청
      await axios.post(`${serverUrl}/resume/${transferId}`);

      // ref 상태 먼저 초기화
      isPausedRef.current = false;
      isUploadingRef.current = true;
      // UI 상태도 초기화
      setIsPaused(false);
      abortControllerRef.current = new AbortController();

      // 이어서 업로드 시작 - 클라이언트 정보 우선 사용
      const chunkSize = 3 * 1024 * 1024; // 3MB
      const serverNextChunk = Math.floor(uploaded_bytes / chunkSize);
      const clientLastChunk = lastSuccessfulChunkRef.current;
      const totalChunks = Math.ceil(selectedFile.size / chunkSize);

      console.log('Resume calculation:', {
        serverNextChunk,
        clientLastChunk,
        totalChunks,
        fileSize: selectedFile.size,
        chunkSize,
      });

      // 클라이언트 정보가 유효하면 그것을 우선 사용
      let safeNextChunk;
      if (clientLastChunk >= 0 && clientLastChunk < totalChunks) {
        // 클라이언트가 추적하는 마지막 성공 청크의 다음 청크부터 시작
        safeNextChunk = clientLastChunk + 1;
        console.log('Using client tracking info for resume:', {
          clientLastChunk,
          nextChunk: safeNextChunk,
        });
      } else {
        // 클라이언트 정보가 없거나 유효하지 않으면 서버 정보 사용
        safeNextChunk = Math.max(0, serverNextChunk);
        console.log('Using server info for resume:', {
          serverNextChunk,
          safeNextChunk,
        });
      }

      // 만약 계산된 시작 청크가 이미 전체 청크 수를 넘었다면 완료 처리
      if (safeNextChunk >= totalChunks) {
        console.log('Calculated start chunk exceeds total chunks - marking as complete');
        setUploadComplete(true);
        setShowSuccess(true);
        return;
      }

      console.log('Final resuming upload details:', {
        uploaded_bytes,
        total_bytes,
        chunkSize,
        serverNextChunk,
        clientLastChunk,
        safeNextChunk,
        totalChunks,
        isPausedRef: isPausedRef.current,
        isUploadingRef: isUploadingRef.current,
      });

      // 업로드 재개 (isResume = true로 설정)
      await uploadWithRetry(safeNextChunk, 0, true);
    } catch (err) {
      console.error('Resume error:', err);
      setError('전송 재개 실패: ' + err.message);
      isPausedRef.current = true;
      isUploadingRef.current = false;
      setIsPaused(true);
    }
  };

  const uploadWithRetry = async (startChunk = 0, retryCount = 0, isResume = false) => {
    if (!selectedFile || !uploadPath) {
      setError('파일과 경로를 모두 선택해주세요.');
      return;
    }

    if (!isUploadingRef.current) {
      console.log('Upload is not active');
      return;
    }

    if (isPausedRef.current) {
      console.log('Upload is paused', {
        isPausedRef: isPausedRef.current,
        isUploadingRef: isUploadingRef.current,
      });
      return;
    }

    console.log('Starting upload with chunk:', startChunk, {
      isPausedRef: isPausedRef.current,
      isUploadingRef: isUploadingRef.current,
      isResume,
    });
    setUploadComplete(false);
    setError('');
    const maxRetries = 3;
    const chunkSize = 3 * 1024 * 1024; // 3MB (2MB에서 증가)
    const totalChunks = Math.ceil(selectedFile.size / chunkSize);
    let uploadedChunks = startChunk;

    // 속도 계산을 위한 전체 업로드 시작 시간
    let uploadStartTime = Date.now();
    let initialUploadedBytes = startChunk * chunkSize;

    try {
      // 새로운 AbortController 생성
      if (!abortControllerRef.current) {
        abortControllerRef.current = new AbortController();
      }

      // 경로 정규화
      const normalizedPath = normalizePath(uploadPath);
      const fullPath = `/home/${normalizedPath}/${selectedFile.name}`;
      const encodedPath = encodeURIComponent(fullPath);

      console.log('Upload details:', {
        path: fullPath,
        totalChunks,
        fileSize: selectedFile.size,
        chunkSize,
        startChunk,
        isPausedRef: isPausedRef.current,
        isUploadingRef: isUploadingRef.current,
        isResume,
      });

      // 첫 번째 청크이고 재개가 아닌 경우에만 새로운 업로드 시작
      if (startChunk === 0 && !isResume) {
        console.log('Preparing first chunk upload');
        const start = 0;
        const end = Math.min(chunkSize, selectedFile.size);
        const firstChunk = selectedFile.slice(start, end);

        console.log('First chunk details:', {
          start,
          end,
          chunkSize: firstChunk.size,
        });

        const formData = new FormData();
        formData.append('file', firstChunk, selectedFile.name);

        try {
          console.log('Sending first chunk request...');
          const response = await axios.post(`${serverUrl}/upload/${encodedPath}`, formData, {
            headers: {
              'Content-Type': 'multipart/form-data',
              'Content-Range': `bytes ${start}-${end - 1}/${selectedFile.size}`,
            },
            signal: abortControllerRef.current.signal,
            onUploadProgress: (progressEvent) => {
              if (!isUploadingRef.current || isPausedRef.current) {
                console.log('Upload progress update skipped - upload not active or paused');
                return;
              }
              const loaded = progressEvent.loaded;
              const total = progressEvent.total;
              const chunkProgress = (loaded / total) * (100 / totalChunks);
              console.log('First chunk progress:', {
                loaded,
                total,
                chunkProgress,
                percentage: ((loaded / total) * 100).toFixed(2) + '%',
              });
              setUploadProgress(chunkProgress);
            },
          });

          console.log('First chunk upload response:', response.data);

          if (!response.data.transfer_id) {
            throw new Error('No transfer ID received from server');
          }

          setTransferId(response.data.transfer_id);
          uploadedChunks++;
          formData.delete('file');

          // 첫 번째 청크 성공 기록
          lastSuccessfulChunkRef.current = 0;
        } catch (err) {
          if (err.name === 'CanceledError') {
            console.log('First chunk upload canceled');
            return;
          }
          console.error('First chunk upload error:', err);
          if (err.response) {
            console.error('Server response:', err.response.data);
            console.error('Server status:', err.response.status);
          }
          throw new Error(`첫 번째 청크 업로드 실패: ${err.message}`);
        }
      }

      // 나머지 청크 업로드
      // 새 업로드: 1번 청크부터 (0번 청크는 이미 업로드됨)
      // 재개: startChunk부터 (서버에서 받은 다음 청크 번호)
      const actualStartChunk = isResume ? startChunk : startChunk === 0 ? 1 : startChunk;
      console.log('Actual start chunk:', actualStartChunk, 'isResume:', isResume);

      for (let i = actualStartChunk; i < totalChunks; i++) {
        if (!isUploadingRef.current || isPausedRef.current) {
          console.log('Upload stopped at chunk:', i, {
            isUploadingRef: isUploadingRef.current,
            isPausedRef: isPausedRef.current,
          });
          setLastUploadedChunk(uploadedChunks);
          return;
        }

        console.log(`Preparing chunk ${i} of ${totalChunks}`);
        const start = i * chunkSize;
        const end = Math.min(start + chunkSize, selectedFile.size);
        const chunk = selectedFile.slice(start, end);

        console.log(`Chunk ${i} details:`, {
          start,
          end,
          chunkSize: chunk.size,
        });

        const chunkFormData = new FormData();
        chunkFormData.append('file', chunk, selectedFile.name);

        try {
          console.log(`Uploading chunk ${i}...`);
          const response = await axios.post(`${serverUrl}/upload/${encodedPath}`, chunkFormData, {
            headers: {
              'Content-Type': 'multipart/form-data',
              'Content-Range': `bytes ${start}-${end - 1}/${selectedFile.size}`,
              'Transfer-Id': transferId,
            },
            signal: abortControllerRef.current.signal,
            onUploadProgress: (progressEvent) => {
              if (!isUploadingRef.current || isPausedRef.current) {
                return;
              }
              const loaded = progressEvent.loaded;
              const total = progressEvent.total;
              const chunkProgress = (loaded / total) * (100 / totalChunks);
              const totalProgress = ((i - 1) * 100) / totalChunks + chunkProgress;
              setUploadProgress(totalProgress);
            },
          });

          // 청크 완료 로그 (필요시에만)
          if (i % 10 === 0) {
            console.log(`Chunk ${i}/${totalChunks} completed`);
          }

          uploadedChunks++;
          setLastUploadedChunk(uploadedChunks);

          // 성공한 청크 번호 기록
          lastSuccessfulChunkRef.current = i;

          // 속도 계산 - 전체 업로드 시간 기준으로 계산 (일부 청크에서만)
          if (i % 5 === 0) {
            // 5개 청크마다 한 번씩만 계산
            const currentTime = Date.now();
            const totalTimeElapsed = (currentTime - uploadStartTime) / 1000; // 초 단위
            const currentUploadedBytes = response.data.uploaded_bytes;
            const totalUploadedBytes = currentUploadedBytes - initialUploadedBytes;

            if (totalTimeElapsed > 1 && totalUploadedBytes > 0) {
              // 최소 1초 경과 후부터 계산
              const speed = totalUploadedBytes / totalTimeElapsed;
              const remaining = response.data.total_bytes - currentUploadedBytes;
              const timeLeft = remaining / speed;

              setUploadSpeed(speed);
              setUploadTimeLeft(formatTime(timeLeft));
            } else {
              setUploadSpeed(0);
              setUploadTimeLeft('계산 중...');
            }
          }

          // 메모리 해제를 위해 청크 데이터 강제 정리
          chunkFormData.delete('file');

          // 가비지 컬렉션 힌트 (10개 청크마다)
          if (i % 10 === 0 && window.gc) {
            window.gc();
          }
        } catch (err) {
          if (err.name === 'CanceledError') {
            console.log(`Chunk ${i} upload canceled`);
            return;
          }
          console.error(`Chunk ${i} upload error:`, err.message);
          if (retryCount < maxRetries) {
            console.log(`Retrying chunk ${i}, attempt ${retryCount + 1}`);
            await new Promise((resolve) => setTimeout(resolve, 1000));
            return uploadWithRetry(uploadedChunks, retryCount + 1, isResume);
          }
          throw new Error(`청크 ${i} 업로드 실패: ${err.message}`);
        }
      }

      console.log('Upload completed');
      setUploadComplete(true);
      setShowSuccess(true);
      setError('');
      isUploadingRef.current = false;
      isPausedRef.current = false;

      // 전송 완료 후 상태 삭제
      try {
        await axios.delete(`${serverUrl}/transfer/${transferId}`);
      } catch (err) {
        console.error('Error deleting transfer status:', err);
      }
    } catch (err) {
      console.error('Upload error:', err);
      if (err.name !== 'CanceledError') {
        setError(err.message || '업로드 실패');
      }
      isUploadingRef.current = false;
      isPausedRef.current = false;
    } finally {
      if (!isPausedRef.current) {
        abortControllerRef.current = null;
      }
    }
  };

  const downloadWithRetry = async (retryCount = 0) => {
    if (!downloadPath) {
      setError('다운로드 경로를 입력해주세요.');
      return;
    }

    console.log('Starting download:', downloadPath);
    const maxRetries = 3;

    try {
      // 다운로드 진행률 초기화
      setDownloadProgress(0);
      setDownloadSpeed(0);
      setDownloadTimeLeft('');
      setError('');

      // 경로 인코딩
      const encodedPath = encodeURIComponent(downloadPath);
      const downloadUrl = `${serverUrl}/download/${encodedPath}`;

      console.log('Download URL:', downloadUrl);

      // 큰 파일의 경우 브라우저의 기본 다운로드 기능 사용
      // 새 창에서 다운로드 URL 열기
      const link = document.createElement('a');
      link.href = downloadUrl;
      link.download = downloadPath.split('/').pop() || 'downloaded_file';
      link.style.display = 'none';

      console.log('Creating direct download link...');
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);

      console.log('Download initiated through browser');

      // 다운로드 시작을 사용자에게 알림
      setError('');
      setDownloadProgress(100); // 진행률을 100%로 설정하여 완료 표시

      // 몇 초 후 진행률 초기화
      setTimeout(() => {
        setDownloadProgress(0);
      }, 3000);
    } catch (err) {
      console.error('Download error:', err);

      if (retryCount < maxRetries) {
        console.log(`Retrying download, attempt ${retryCount + 1}`);
        await new Promise((resolve) => setTimeout(resolve, 1000));
        return downloadWithRetry(retryCount + 1);
      }

      setError('다운로드 실패: ' + err.message);
    }
  };

  const streamDownload = async () => {
    if (!downloadPath) {
      setError('다운로드 경로를 입력해주세요.');
      return;
    }

    console.log('Starting stream download:', downloadPath);

    try {
      // 다운로드 진행률 초기화
      setDownloadProgress(0);
      setDownloadSpeed(0);
      setDownloadTimeLeft('');
      setError('');

      // 경로 인코딩
      const encodedPath = encodeURIComponent(downloadPath);
      const streamUrl = `${serverUrl}/stream-download/${encodedPath}`;

      console.log('Stream download URL:', streamUrl);

      // 스트리밍 다운로드 링크 생성
      const link = document.createElement('a');
      link.href = streamUrl;
      link.download = downloadPath.split('/').pop() || 'downloaded_file';
      link.style.display = 'none';

      console.log('Creating stream download link...');
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);

      console.log('Stream download initiated');

      // 다운로드 시작을 사용자에게 알림
      setError('');
      setDownloadProgress(100);

      // 몇 초 후 진행률 초기화
      setTimeout(() => {
        setDownloadProgress(0);
      }, 3000);
    } catch (err) {
      console.error('Stream download error:', err);
      setError('스트리밍 다운로드 실패: ' + err.message);
    }
  };

  const formatTime = (seconds) => {
    if (!isFinite(seconds) || seconds < 0 || seconds === 0) {
      return '계산 중...';
    }
    if (seconds < 1) {
      return '1초 미만';
    }
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);
    return `${hours}시간 ${minutes}분 ${secs}초`;
  };

  const formatSpeed = (bytesPerSecond) => {
    const units = ['B/s', 'KB/s', 'MB/s', 'GB/s'];
    let speed = bytesPerSecond;
    let unitIndex = 0;
    while (speed >= 1024 && unitIndex < units.length - 1) {
      speed /= 1024;
      unitIndex++;
    }
    return `${speed.toFixed(2)} ${units[unitIndex]}`;
  };

  return (
    <div className="p-4 bg-white rounded-lg shadow">
      <div className="mb-6">
        <Typography variant="h6" className="mb-4">
          파일 업로드
        </Typography>
        <input type="file" onChange={handleFileSelect} className="mb-4" />
        <TextField
          fullWidth
          label="서버 저장 경로"
          value={uploadPath}
          onChange={(e) => setUploadPath(e.target.value)}
          className="mb-4"
        />
        <Button
          variant="contained"
          color="primary"
          onClick={handleUploadClick}
          disabled={!selectedFile || !uploadPath || isPaused}
          className="mr-2"
        >
          업로드
        </Button>
        {isPaused ? (
          <Button variant="contained" color="secondary" onClick={handleResume} className="mr-2">
            재개
          </Button>
        ) : (
          <Button
            variant="contained"
            color="secondary"
            onClick={handlePause}
            disabled={!transferId}
            className="mr-2"
          >
            일시 중지
          </Button>
        )}
        {uploadProgress > 0 && (
          <Box className="mt-4">
            <LinearProgress
              variant="determinate"
              value={uploadProgress}
              color={uploadComplete ? 'success' : 'primary'}
            />
            <Box className="flex items-center mt-2">
              <Typography variant="body2" className="mr-2">
                진행률: {uploadProgress.toFixed(1)}%
              </Typography>
              {uploadComplete && <CheckCircleIcon color="success" className="ml-2" />}
            </Box>
            <Typography variant="body2">속도: {formatSpeed(uploadSpeed)}</Typography>
            <Typography variant="body2">예상 남은 시간: {uploadTimeLeft}</Typography>
          </Box>
        )}
      </div>

      <div className="mt-8">
        <Typography variant="h6" className="mb-4">
          파일 다운로드
        </Typography>
        <TextField
          fullWidth
          label="다운로드 파일 경로"
          value={downloadPath}
          onChange={(e) => setDownloadPath(e.target.value)}
          className="mb-4"
          placeholder="예: /home/ola/colcon_ws/filename.zip"
          helperText="서버에 있는 파일의 절대 경로를 입력하세요"
        />
        <Button
          variant="contained"
          color="primary"
          onClick={downloadWithRetry}
          disabled={!downloadPath || isPaused}
          className="mr-2"
        >
          다운로드
        </Button>
        <Button
          variant="outlined"
          color="secondary"
          onClick={streamDownload}
          disabled={!downloadPath || isPaused}
          className="mr-2"
        >
          스트리밍 다운로드
        </Button>
        {isPaused ? (
          <Button variant="contained" color="secondary" onClick={handleResume} className="mr-2">
            재개
          </Button>
        ) : (
          <Button
            variant="contained"
            color="secondary"
            onClick={handlePause}
            disabled={!transferId}
            className="mr-2"
          >
            일시 중지
          </Button>
        )}
        {downloadProgress > 0 && (
          <Box className="mt-4">
            <LinearProgress variant="determinate" value={downloadProgress} />
            <Typography variant="body2" className="mt-2">
              진행률: {downloadProgress.toFixed(1)}%
            </Typography>
            <Typography variant="body2">속도: {formatSpeed(downloadSpeed)}</Typography>
            <Typography variant="body2">예상 남은 시간: {downloadTimeLeft}</Typography>
          </Box>
        )}
      </div>

      {error && (
        <Alert severity="error" className="mt-4">
          <ErrorIcon className="mr-2" />
          {error}
        </Alert>
      )}

      <Snackbar
        open={showSuccess}
        autoHideDuration={6000}
        onClose={() => setShowSuccess(false)}
        message="파일 업로드가 완료되었습니다"
      />

      <Dialog open={showOverwriteDialog} onClose={handleOverwriteCancel}>
        <DialogTitle>파일 덮어쓰기</DialogTitle>
        <DialogContent>
          <Typography>해당 경로에 이미 동일한 이름의 파일이 있습니다. 덮어쓰시겠습니까?</Typography>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleOverwriteCancel} color="primary">
            취소
          </Button>
          <Button onClick={handleOverwriteConfirm} color="primary" variant="contained">
            덮어쓰기
          </Button>
        </DialogActions>
      </Dialog>
    </div>
  );
}
