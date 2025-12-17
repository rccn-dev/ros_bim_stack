import pytest
import os
from ros_speckle_bridge.speckle_client import SpeckleClient

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
        host = os.getenv("SPECKLE_HOST", "https://app.speckle.systems")
        
        # Initialize client (will use SPECKLE_TOKEN from env)
        client = SpeckleClient(host=host)
        
        # This triggers the actual authentication check
        user = client.client.active_user.get()
        
        assert user is not None
        assert user.email is not None
        print(f"\nSuccessfully authenticated as: {user.name} ({user.email})")

    def test_fetch_public_stream(self):
        """
        Real network test: Can we fetch a real stream?
        """
        # Use a specific stream ID if provided
        stream_id = os.getenv("TEST_STREAM_ID")
        
        if not stream_id:
            pytest.skip("TEST_STREAM_ID not set. Skipping live stream fetch.")
            
        host = os.getenv("SPECKLE_HOST", "https://app.speckle.systems")
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
