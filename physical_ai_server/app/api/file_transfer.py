from fastapi import FastAPI, HTTPException, UploadFile, File, Header, Response
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
import os
import uuid
import shutil
from typing import Optional, Dict
import uvicorn
import logging
from pathlib import Path
from urllib.parse import unquote
import json

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI()

# CORS 설정
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 실제 운영 환경에서는 특정 도메인만 허용하도록 수정
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 전송 상태 저장소
transfer_status: Dict[str, dict] = {}

def save_transfer_status():
    """전송 상태를 파일에 저장"""
    try:
        with open('transfer_status.json', 'w') as f:
            json.dump(transfer_status, f)
    except Exception as e:
        logger.error(f"Error saving transfer status: {str(e)}")

def load_transfer_status():
    """저장된 전송 상태를 로드"""
    try:
        if os.path.exists('transfer_status.json'):
            with open('transfer_status.json', 'r') as f:
                return json.load(f)
    except Exception as e:
        logger.error(f"Error loading transfer status: {str(e)}")
    return {}

# 서버 시작 시 저장된 상태 로드
transfer_status = load_transfer_status()

def sanitize_path(file_path: str) -> str:
    """파일 경로를 안전하게 처리하는 함수"""
    # URL 디코딩
    decoded_path = unquote(file_path)
    
    # 경로 정규화
    normalized_path = os.path.normpath(decoded_path)
    
    # 절대 경로가 아닌 경우 에러
    if not os.path.isabs(normalized_path):
        raise HTTPException(status_code=400, detail="Absolute path is required")
    
    return normalized_path

@app.post("/upload/{file_path:path}")
async def upload_file(
    file_path: str,
    file: UploadFile = File(...),
    content_range: Optional[str] = Header(None),
    transfer_id: Optional[str] = Header(None)
):
    try:
        logger.info(f"Received upload request for path: {file_path}")
        logger.info(f"File details - filename: {file.filename}, content_type: {file.content_type}")
        
        if not file:
            raise HTTPException(status_code=400, detail="No file provided")
            
        full_path = sanitize_path(file_path)
        os.makedirs(os.path.dirname(full_path), exist_ok=True)
        
        # 전송 ID 생성 또는 사용
        if not transfer_id:
            transfer_id = str(uuid.uuid4())
            transfer_status[transfer_id] = {
                "paused": False,
                "complete": False,
                "uploaded_bytes": 0,
                "total_bytes": 0,
                "file_path": full_path
            }
        elif transfer_id not in transfer_status:
            transfer_status[transfer_id] = {
                "paused": False,
                "complete": False,
                "uploaded_bytes": 0,
                "total_bytes": 0,
                "file_path": full_path
            }

        # Content-Range 헤더 파싱
        start_byte = 0
        total_bytes = 0
        if content_range:
            try:
                range_header = content_range.split(' ')[1]
                start_byte = int(range_header.split('-')[0])
                total_bytes = int(range_header.split('/')[1])
                transfer_status[transfer_id]["total_bytes"] = total_bytes
                logger.info(f"Content-Range: start={start_byte}, total={total_bytes}")
            except Exception as e:
                logger.error(f"Error parsing Content-Range: {str(e)}")
                raise HTTPException(status_code=400, detail="Invalid Content-Range header")

        # 파일 저장
        try:
            content = await file.read()
            if not content:
                raise HTTPException(status_code=400, detail="Empty file content")
                
            # 파일을 바이너리 모드로 열고 특정 위치에 쓰기
            with open(full_path, 'r+b' if os.path.exists(full_path) else 'wb') as f:
                f.seek(start_byte)  # 정확한 위치로 이동
                f.write(content)
                
            # 업로드된 바이트 수 업데이트 - 현재 청크의 끝 위치로 계산
            current_end_byte = start_byte + len(content)
            
            # 순차적 업로드인지 확인 (올바른 경우에만 업데이트)
            if current_end_byte <= total_bytes:
                # 기존 업로드된 바이트와 비교하여 더 큰 값 사용 (중복 업로드 방지)
                transfer_status[transfer_id]["uploaded_bytes"] = max(
                    transfer_status[transfer_id]["uploaded_bytes"], 
                    current_end_byte
                )
            else:
                # 비정상적인 경우 - 현재 청크 끝 위치가 전체 파일 크기를 초과
                logger.warning(f"Current end byte {current_end_byte} exceeds total bytes {total_bytes}")
                transfer_status[transfer_id]["uploaded_bytes"] = min(current_end_byte, total_bytes)
            
            save_transfer_status()
                
            logger.info(f"File successfully uploaded to {full_path}")
            logger.info(f"Chunk range: {start_byte}-{current_end_byte-1}")
            logger.info(f"Current uploaded bytes: {transfer_status[transfer_id]['uploaded_bytes']}")
            logger.info(f"Total bytes: {total_bytes}")
            
            # 진행률 계산
            progress = (transfer_status[transfer_id]["uploaded_bytes"] / total_bytes * 100) if total_bytes > 0 else 0
            logger.info(f"Upload progress: {progress:.2f}%")
            
        except Exception as e:
            logger.error(f"Error while saving file: {str(e)}")
            raise HTTPException(status_code=500, detail=f"Error saving file: {str(e)}")

        return {
            "transfer_id": transfer_id,
            "path": full_path,
            "uploaded_bytes": transfer_status[transfer_id]["uploaded_bytes"],
            "total_bytes": total_bytes,
            "progress": progress
        }

    except HTTPException as he:
        logger.error(f"HTTP Exception: {str(he)}")
        raise he
    except Exception as e:
        logger.error(f"Unexpected error during upload: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/download/{file_path:path}")
async def download_file(
    file_path: str,
    range: Optional[str] = Header(None),
    transfer_id: Optional[str] = Header(None)
):
    try:
        logger.info(f"Download request received for path: {file_path}")
        logger.info(f"Range header: {range}")
        logger.info(f"Transfer ID: {transfer_id}")
        
        full_path = sanitize_path(file_path)
        logger.info(f"Full path after sanitization: {full_path}")
        
        if not os.path.exists(full_path):
            logger.error(f"File not found: {full_path}")
            raise HTTPException(status_code=404, detail="File not found")

        # 파일 크기 확인
        file_size = os.path.getsize(full_path)
        logger.info(f"File size: {file_size} bytes")

        # 전송 ID 생성 또는 사용
        if not transfer_id:
            transfer_id = str(uuid.uuid4())
            transfer_status[transfer_id] = {"paused": False, "complete": False}

        # Range 헤더 처리
        start = 0
        if range:
            try:
                start = int(range.split("=")[1].split("-")[0])
                logger.info(f"Range request start: {start}")
            except Exception as e:
                logger.error(f"Error parsing range header: {str(e)}")

        logger.info(f"Creating file response for: {full_path}")
        logger.info(f"File basename: {os.path.basename(file_path)}")

        # 파일 응답 생성
        response = FileResponse(
            full_path,
            media_type="application/octet-stream",
            filename=os.path.basename(file_path)
        )
        response.headers["Content-Range"] = f"bytes {start}-{file_size-1}/{file_size}"
        response.headers["Transfer-Id"] = transfer_id
        response.headers["Content-Length"] = str(file_size)
        response.headers["Accept-Ranges"] = "bytes"

        logger.info(f"Response headers set: Content-Length={file_size}, Transfer-Id={transfer_id}")
        return response

    except HTTPException as he:
        logger.error(f"HTTP Exception in download: {str(he)}")
        raise he
    except Exception as e:
        logger.error(f"Unexpected error in download: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/pause/{transfer_id}")
async def pause_transfer(transfer_id: str):
    if transfer_id not in transfer_status:
        raise HTTPException(status_code=404, detail="Transfer not found")
    
    transfer_status[transfer_id]["paused"] = True
    return {"status": "paused"}

@app.post("/resume/{transfer_id}")
async def resume_transfer(transfer_id: str):
    if transfer_id not in transfer_status:
        raise HTTPException(status_code=404, detail="Transfer not found")
    
    transfer_status[transfer_id]["paused"] = False
    return {"status": "resumed"}

@app.get("/transfer-status/{transfer_id}")
async def get_transfer_status(transfer_id: str):
    """전송 상태 조회"""
    if transfer_id not in transfer_status:
        raise HTTPException(status_code=404, detail="Transfer not found")
    
    return transfer_status[transfer_id]

@app.delete("/transfer/{transfer_id}")
async def delete_transfer(transfer_id: str):
    """전송 상태 삭제"""
    if transfer_id in transfer_status:
        del transfer_status[transfer_id]
        save_transfer_status()
    return {"status": "deleted"}

@app.head("/check-file/{file_path:path}")
async def check_file_exists(file_path: str):
    try:
        full_path = sanitize_path(file_path)
        exists = os.path.exists(full_path)
        if exists:
            return Response(status_code=200)
        return Response(status_code=404)
    except Exception as e:
        logger.error(f"Error checking file existence: {str(e)}")
        return Response(status_code=500)

@app.get("/stream-download/{file_path:path}")
async def stream_download_file(
    file_path: str,
    range: Optional[str] = Header(None)
):
    """스트리밍 다운로드 엔드포인트"""
    try:
        logger.info(f"Stream download request received for path: {file_path}")
        
        full_path = sanitize_path(file_path)
        logger.info(f"Full path after sanitization: {full_path}")
        
        if not os.path.exists(full_path):
            logger.error(f"File not found: {full_path}")
            raise HTTPException(status_code=404, detail="File not found")

        file_size = os.path.getsize(full_path)
        logger.info(f"File size: {file_size} bytes")
        
        # Range 헤더 처리
        start = 0
        end = file_size - 1
        
        if range:
            range_match = range.replace('bytes=', '').split('-')
            start = int(range_match[0]) if range_match[0] else 0
            end = int(range_match[1]) if range_match[1] else file_size - 1
            logger.info(f"Range request: {start}-{end}")
        
        # 스트리밍 응답 생성
        def iterfile():
            with open(full_path, 'rb') as file_like:
                file_like.seek(start)
                remaining = end - start + 1
                while remaining:
                    chunk_size = min(8192, remaining)  # 8KB 청크
                    chunk = file_like.read(chunk_size)
                    if not chunk:
                        break
                    remaining -= len(chunk)
                    yield chunk
        
        headers = {
            'Content-Range': f'bytes {start}-{end}/{file_size}',
            'Accept-Ranges': 'bytes',
            'Content-Length': str(end - start + 1),
            'Content-Disposition': f'attachment; filename="{os.path.basename(file_path)}"'
        }
        
        status_code = 206 if range else 200
        
        return StreamingResponse(
            iterfile(),
            status_code=status_code,
            headers=headers,
            media_type='application/octet-stream'
        )
        
    except HTTPException as he:
        logger.error(f"HTTP Exception in stream download: {str(he)}")
        raise he
    except Exception as e:
        logger.error(f"Unexpected error in stream download: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000) 