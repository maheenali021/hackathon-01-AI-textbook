"""
Hugging Face Space App for RAG Agent
This file serves as the entry point for Hugging Face deployment
"""
import os
import sys
from pathlib import Path

# Add the backend directory to the path so we can import rag_agent
backend_dir = Path(__file__).parent
if str(backend_dir) not in sys.path:
    sys.path.insert(0, str(backend_dir))

from rag_agent.main import app

# The FastAPI app instance that Hugging Face will serve
# This is the same app defined in rag_agent.main
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 8000)))