#!/usr/bin/env python3
"""
Script to run the RAG Agent server
This is specifically designed for Hugging Face deployment
"""
import os
import sys
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Add the backend directory to the path so we can import rag_agent
current_dir = Path(__file__).parent
if str(current_dir) not in sys.path:
    sys.path.insert(0, str(current_dir))

def run_server():
    """Run the RAG Agent server"""
    try:
        # Import the app after setting up the path
        from rag_agent.main import app

        # Get port from environment (Hugging Face sets this)
        port = int(os.environ.get("PORT", 8000))

        # Import uvicorn to run the server
        import uvicorn

        print(f"Starting RAG Agent server on port {port}...")
        print("API endpoints available:")
        print("  - GET /")
        print("  - POST /api/v1/agent/chat")
        print("  - POST /api/v1/agent/session")
        print("  - GET /api/v1/agent/session/{session_id}")
        print("  - DELETE /api/v1/agent/session/{session_id}")
        print("  - GET /api/v1/agent/health")

        uvicorn.run(
            app,
            host="0.0.0.0",
            port=port,
            log_level="info"
        )

    except ImportError as e:
        print(f"Error importing modules: {e}")
        print("Make sure all dependencies are installed:")
        print("pip install -r requirements.txt")
        sys.exit(1)
    except ValueError as e:
        if "Missing required environment variables" in str(e):
            print("Environment configuration error:")
            print(f"{e}")
            print("\nPlease set the required environment variables:")
            print("- GEMINI_API_KEY: Your Google Gemini API key")
            print("- GEMINI_MODEL: Google Gemini model (default: gemini-pro)")
            print("- QDRANT_URL: URL for Qdrant vector database")
            print("- QDRANT_API_KEY: API key for Qdrant database")
            print("- QDRANT_COLLECTION_NAME: Name of the collection in Qdrant")
            sys.exit(1)
        else:
            raise
    except Exception as e:
        print(f"Error starting server: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    run_server()