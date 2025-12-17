#!/usr/bin/env python3
"""
Speckle Client Module
Handles authentication, API communication, and data fetching from Speckle servers
"""

import os
import logging
from typing import Optional, Dict, Any, List
from specklepy.api.client import SpeckleClient as SpeckleAPIClient
from specklepy.api.credentials import get_account_from_token
from specklepy.transports.server import ServerTransport
from specklepy.api import operations


class SpeckleClient:
    """Wrapper around specklepy for BIM data retrieval with authentication"""

    def __init__(self, host: str = "https://app.speckle.systems"):
        """
        Initialize Speckle client
        
        Args:
            host: Speckle server URL
            
        Raises:
            RuntimeError: If SPECKLE_TOKEN environment variable is not set
        """
        self.logger = logging.getLogger(__name__)
        self.host = host
        self.token = os.getenv("SPECKLE_TOKEN")
        
        if not self.token:
            raise RuntimeError(
                "SPECKLE_TOKEN environment variable not set. "
                "Please set your Speckle Personal Access Token."
            )
        
        self.client: Optional[SpeckleAPIClient] = None
        self.account = None
        self._authenticate()

    def _authenticate(self):
        """Authenticate with Speckle server using token"""
        try:
            self.logger.info(f"Authenticating with Speckle server: {self.host}")
            self.account = get_account_from_token(self.token, self.host)
            self.client = SpeckleAPIClient(host=self.host)
            self.client.authenticate_with_token(self.token)
            
            # Verify authentication
            try:
                user = self.client.active_user.get()
                self.logger.info(f"Successfully authenticated as: {user.email} ({user.name})")
            except Exception as auth_err:
                self.logger.warning(f"Authenticated but failed to fetch user info: {auth_err}")
                
        except Exception as e:
            self.logger.error(f"Authentication failed: {e}")
            raise RuntimeError(f"Failed to authenticate with Speckle: {e}")

    def get_stream(self, stream_id: str) -> Dict[str, Any]:
        """
        Fetch stream metadata
        
        Args:
            stream_id: The stream identifier
            
        Returns:
            Stream metadata dictionary
        """
        try:
            # Support for specklepy 2.19+ (Project API)
            if hasattr(self.client, "project"):
                return self.client.project.get(stream_id)
            return self.client.stream.get(stream_id)
        except Exception as e:
            self.logger.error(f"Failed to fetch stream {stream_id}: {e}")
            raise

    def get_commit(self, stream_id: str, commit_id: str = "latest") -> Dict[str, Any]:
        """
        Fetch commit metadata
        
        Args:
            stream_id: The stream identifier
            commit_id: The commit ID or "latest" for most recent
            
        Returns:
            Commit metadata dictionary
        """
        try:
            if commit_id == "latest":
                # Try to get "main" model versions
                # Note: get_with_versions takes (model_id, project_id)
                # We assume model name is "main" which is default
                # First we need to find the model ID for "main"
                project = self.client.project.get_with_models(stream_id)
                main_model = next((m for m in project.models.items if m.name == "main"), None)
                
                if not main_model:
                     # Fallback to first model
                     if project.models.items:
                         main_model = project.models.items[0]
                     else:
                         raise ValueError(f"No models found in project {stream_id}")

                versions = self.client.version.get_versions(main_model.id, stream_id, limit=1)
                if not versions.items:
                     raise ValueError(f"No versions found for model '{main_model.name}' in project {stream_id}")
                commit = versions.items[0]
            else:
                # version.get takes (version_id, project_id)
                commit = self.client.version.get(commit_id, stream_id)
            
            self.logger.info(f"Retrieved commit: {commit.id} - {getattr(commit, 'message', 'No message')}")
            return commit
        except Exception as e:
            self.logger.error(f"Failed to fetch commit: {e}")
            raise

    def receive_objects(self, stream_id: str, commit_id: str = "latest") -> Any:
        """
        Receive all objects from a commit
        
        Args:
            stream_id: The stream identifier
            commit_id: The commit ID or "latest"
            
        Returns:
            Speckle Base object containing all data
        """
        try:
            commit = self.get_commit(stream_id, commit_id)
            transport = ServerTransport(client=self.client, stream_id=stream_id)
            
            self.logger.info(f"Receiving objects from commit {commit.id}...")
            self.logger.info(f"Commit object attributes: {dir(commit)}")
            # Try to handle both Commit (referencedObject) and Version (referencedObject or similar)
            ref_obj = getattr(commit, 'referencedObject', None)
            if not ref_obj:
                 # Try snake_case for newer specklepy versions
                 ref_obj = getattr(commit, 'referenced_object', None)
            
            if not ref_obj:
                 raise AttributeError(f"Could not find referenced object in commit/version object. Available attributes: {dir(commit)}")

            obj = operations.receive(ref_obj, transport)
            self.logger.info("Objects received successfully")
            
            return obj
        except Exception as e:
            self.logger.error(f"Failed to receive objects: {e}")
            raise

    def list_streams(self, limit: int = 25) -> List[Dict[str, Any]]:
        """
        List available streams for the authenticated user
        
        Args:
            limit: Maximum number of streams to return
            
        Returns:
            List of stream metadata dictionaries
        """
        try:
            # Support for specklepy 2.19+ (Project API)
            if hasattr(self.client, "active_user"):
                 return self.client.active_user.get_projects(limit=limit).items
            return self.client.stream.list(limit=limit)
        except Exception as e:
            self.logger.error(f"Failed to list streams: {e}")
            raise

    def is_online(self) -> bool:
        """
        Check if the Speckle server is reachable
        
        Returns:
            True if server is reachable, False otherwise
        """
        try:
            self.client.account.get()
            return True
        except Exception:
            return False
