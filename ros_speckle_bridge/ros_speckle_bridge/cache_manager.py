#!/usr/bin/env python3
"""
Cache Manager Module
Handles offline resilience by caching Speckle data locally
"""

import os
import json
import logging
from pathlib import Path
from typing import Optional, Any, Dict
from datetime import datetime


class CacheManager:
    """Manages local caching of Speckle BIM data for offline operation"""

    def __init__(self, cache_dir: Optional[str] = None):
        """
        Initialize cache manager
        
        Args:
            cache_dir: Directory for cache storage (defaults to ~/.ros/speckle_cache/)
        """
        self.logger = logging.getLogger(__name__)
        
        if cache_dir is None:
            home = os.path.expanduser("~")
            cache_dir = os.path.join(home, ".ros", "speckle_cache")
        
        self.cache_dir = Path(cache_dir)
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        self.logger.info(f"Cache directory: {self.cache_dir}")

    def _get_cache_path(self, stream_id: str, commit_id: str) -> Path:
        """
        Get cache file path for a specific stream and commit
        
        Args:
            stream_id: Speckle stream ID
            commit_id: Speckle commit ID
            
        Returns:
            Path to cache file
        """
        filename = f"{stream_id}_{commit_id}.json"
        return self.cache_dir / filename

    def _get_metadata_path(self, stream_id: str, commit_id: str) -> Path:
        """Get metadata file path for cache entry"""
        filename = f"{stream_id}_{commit_id}_metadata.json"
        return self.cache_dir / filename

    def has_cache(self, stream_id: str, commit_id: str) -> bool:
        """
        Check if cache exists for given stream and commit
        
        Args:
            stream_id: Speckle stream ID
            commit_id: Speckle commit ID
            
        Returns:
            True if cache exists, False otherwise
        """
        cache_path = self._get_cache_path(stream_id, commit_id)
        exists = cache_path.exists()
        
        if exists:
            self.logger.info(f"Cache found: {cache_path}")
        else:
            self.logger.info(f"No cache found for {stream_id}/{commit_id}")
        
        return exists

    def save_cache(self, stream_id: str, commit_id: str, data: Any) -> bool:
        """
        Save Speckle data to cache
        
        Args:
            stream_id: Speckle stream ID
            commit_id: Speckle commit ID
            data: Data to cache (must be JSON serializable)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            cache_path = self._get_cache_path(stream_id, commit_id)
            metadata_path = self._get_metadata_path(stream_id, commit_id)
            
            # Convert Speckle objects to dictionary for JSON serialization
            cache_data = self._serialize_speckle_object(data)
            
            # Save main cache file
            with open(cache_path, 'w') as f:
                json.dump(cache_data, f, indent=2)
            
            # Save metadata
            metadata = {
                "stream_id": stream_id,
                "commit_id": commit_id,
                "cached_at": datetime.now().isoformat(),
                "cache_version": "1.0"
            }
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            self.logger.info(f"Cache saved: {cache_path}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to save cache: {e}")
            return False

    def load_cache(self, stream_id: str, commit_id: str) -> Optional[Dict]:
        """
        Load data from cache
        
        Args:
            stream_id: Speckle stream ID
            commit_id: Speckle commit ID
            
        Returns:
            Cached data or None if not available
        """
        try:
            cache_path = self._get_cache_path(stream_id, commit_id)
            
            if not cache_path.exists():
                self.logger.warning(f"Cache file not found: {cache_path}")
                return None
            
            with open(cache_path, 'r') as f:
                data = json.load(f)
            
            self.logger.info(f"Cache loaded: {cache_path}")
            return data
            
        except Exception as e:
            self.logger.error(f"Failed to load cache: {e}")
            return None

    def get_cache_metadata(self, stream_id: str, commit_id: str) -> Optional[Dict]:
        """
        Get cache metadata
        
        Args:
            stream_id: Speckle stream ID
            commit_id: Speckle commit ID
            
        Returns:
            Metadata dictionary or None
        """
        try:
            metadata_path = self._get_metadata_path(stream_id, commit_id)
            
            if not metadata_path.exists():
                return None
            
            with open(metadata_path, 'r') as f:
                metadata = json.load(f)
            
            return metadata
            
        except Exception as e:
            self.logger.error(f"Failed to load metadata: {e}")
            return None

    def _serialize_speckle_object(self, obj: Any) -> Any:
        """
        Recursively serialize Speckle objects to JSON-compatible format
        
        Args:
            obj: Speckle object or primitive
            
        Returns:
            JSON-serializable object
        """
        # Handle None
        if obj is None:
            return None
        
        # Handle primitives
        if isinstance(obj, (str, int, float, bool)):
            return obj
        
        # Handle lists
        if isinstance(obj, list):
            return [self._serialize_speckle_object(item) for item in obj]
        
        # Handle dictionaries
        if isinstance(obj, dict):
            return {k: self._serialize_speckle_object(v) for k, v in obj.items()}
        
        # Handle Speckle Base objects (have __dict__ attribute)
        if hasattr(obj, '__dict__'):
            return self._serialize_speckle_object(obj.__dict__)
        
        # Fallback: try to convert to string
        try:
            return str(obj)
        except Exception:
            return None

    def clear_cache(self, stream_id: Optional[str] = None):
        """
        Clear cache files
        
        Args:
            stream_id: If provided, clear only this stream's cache. 
                      Otherwise, clear all cache.
        """
        try:
            if stream_id:
                pattern = f"{stream_id}_*.json"
                files = list(self.cache_dir.glob(pattern))
            else:
                files = list(self.cache_dir.glob("*.json"))
            
            for file in files:
                file.unlink()
                self.logger.info(f"Removed cache file: {file}")
            
            self.logger.info(f"Cleared {len(files)} cache files")
            
        except Exception as e:
            self.logger.error(f"Failed to clear cache: {e}")
