import pytest
import os
import yaml
from pathlib import Path
from ros_speckle_bridge.speckle_client import SpeckleClient

def get_speckle_host():
    """
    Determine Speckle host from:
    1. Environment variable SPECKLE_HOST
    2. params.yaml configuration
    3. Default fallback
    """
    # 1. Try env var
    env_host = os.getenv("SPECKLE_HOST")
    if env_host:
        return env_host
        
    # 2. Try params.yaml
    try:
        # Assuming this test file is in tests/ and config is in ../config/
        current_dir = Path(__file__).parent
        config_path = current_dir.parent / "config" / "params.yaml"
        
        if config_path.exists():
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                # Navigate: /** -> ros__parameters -> host
                # Note: yaml.safe_load might return None for empty file
                if config:
                    host = config.get('/**', {}).get('ros__parameters', {}).get('host')
                    if host:
                        return host
    except Exception as e:
        print(f"Warning: Could not read params.yaml: {e}")
        
    # 3. Default
    return "https://app.speckle.systems"

# Only run these tests if a real token is present in the environment
@pytest.mark.skipif(
    not os.getenv("SPECKLE_TOKEN") or os.getenv("SPECKLE_TOKEN") == "dummy_token_for_tests",
    reason="Requires real SPECKLE_TOKEN to run live tests"
)
class TestLiveConnection:
    
    def test_authentication_ping(self):
        """
        Real network test: Can we actually authenticate with Speckle?
        """
        host = get_speckle_host()
        print(f"\nConnecting to Speckle Host: {host}")
        
        # Initialize client (will use SPECKLE_TOKEN from env)
        client = SpeckleClient(host=host)
        
        # This triggers the actual authentication check
        user = client.client.active_user.get()
        
        assert user is not None
        assert user.email is not None
        print(f"Successfully authenticated as: {user.name} ({user.email})")

    def test_fetch_public_stream(self):
        """
        Real network test: Can we fetch a real stream?
        """
        # Use a specific stream ID if provided
        stream_id = os.getenv("TEST_STREAM_ID")
        
        if not stream_id:
            pytest.skip("TEST_STREAM_ID not set. Skipping live stream fetch.")
            
        host = get_speckle_host()
        client = SpeckleClient(host=host)
        
        # Try to get the stream/project
        # Note: Specklepy 2.19+ unifies streams as projects
        if hasattr(client.client, "project"):
            stream = client.client.project.get(stream_id)
        else:
            stream = client.client.stream.get(stream_id)
        
        assert stream is not None
        assert stream.id == stream_id
        print(f"\nSuccessfully fetched stream: {stream.name}")
