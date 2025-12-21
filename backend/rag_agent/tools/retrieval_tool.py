"""
Retrieval tool for the RAG Agent system
Implements tool calling to invoke the retrieval pipeline
"""
import requests
import logging
from typing import Dict, Any, List, Optional
from datetime import datetime

from ..config import Config


class RetrievalTool:
    """
    Tool for invoking the retrieval pipeline to get relevant book content
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.config = Config

    def search(self, query: str, filters: Optional[Dict[str, Any]] = None, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for relevant content using the retrieval pipeline

        Args:
            query: Search query text
            filters: Optional metadata filters (e.g., chapter, section)
            top_k: Number of results to return

        Returns:
            List of retrieved content chunks with metadata
        """
        try:
            # Prepare the request to the retrieval API
            payload = {
                "query": query,
                "expected_sources": None,  # Not needed for basic retrieval
                "filters": filters,
                "top_k": top_k
            }

            # Make request to the retrieval validation API
            response = requests.post(
                self.config.RETRIEVAL_API_URL,
                json=payload,
                headers={"Content-Type": "application/json"},
                timeout=self.config.RETRIEVAL_TIMEOUT
            )

            if response.status_code != 200:
                self.logger.error(f"Retrieval API returned status {response.status_code}: {response.text}")
                return []

            # Parse the response
            result = response.json()

            # Extract retrieved results
            retrieved_results = result.get("retrieved_results", [])
            processed_results = []

            for item in retrieved_results:
                processed_item = {
                    "id": item.get("id", ""),
                    "content": item.get("content", ""),
                    "source_url": item.get("source_url", ""),
                    "chapter": item.get("chapter"),
                    "section": item.get("section"),
                    "similarity_score": item.get("similarity_score", 0.0),
                    "confidence_score": item.get("confidence_score", 0.0),
                    "retrieval_timestamp": datetime.now().isoformat(),
                    "metadata": item.get("metadata", {})
                }
                processed_results.append(processed_item)

            self.logger.info(f"Retrieved {len(processed_results)} results for query: {query[:50]}...")
            return processed_results

        except requests.exceptions.Timeout:
            self.logger.error(f"Retrieval API request timed out after {self.config.RETRIEVAL_TIMEOUT} seconds")
            return []
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Error calling retrieval API: {str(e)}")
            return []
        except Exception as e:
            self.logger.error(f"Unexpected error in retrieval tool: {str(e)}")
            return []

    def validate_retrieval_connection(self) -> bool:
        """
        Validate that the retrieval API is accessible
        """
        try:
            # Make a simple request to validate the connection
            test_payload = {
                "query": "test",
                "expected_sources": None,
                "filters": None,
                "top_k": 1
            }

            response = requests.post(
                self.config.RETRIEVAL_API_URL,
                json=test_payload,
                headers={"Content-Type": "application/json"},
                timeout=5  # Short timeout for validation
            )

            return response.status_code == 200
        except:
            return False

    def search_by_chapter(self, query: str, chapter: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search specifically within a chapter

        Args:
            query: Search query text
            chapter: Chapter to search within
            top_k: Number of results to return

        Returns:
            List of retrieved content chunks from the specified chapter
        """
        filters = {"chapter": chapter}
        return self.search(query, filters, top_k)

    def search_with_filters(self, query: str, filters: Dict[str, Any], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search with custom filters

        Args:
            query: Search query text
            filters: Dictionary of metadata filters
            top_k: Number of results to return

        Returns:
            List of retrieved content chunks matching the filters
        """
        return self.search(query, filters, top_k)

    def get_content_by_source_url(self, source_url: str) -> List[Dict[str, Any]]:
        """
        Retrieve content by source URL

        Args:
            source_url: URL of the source document

        Returns:
            List of content chunks from the specified source
        """
        filters = {"source_url": source_url}
        return self.search("all content", filters, top_k=20)  # Get all content from source