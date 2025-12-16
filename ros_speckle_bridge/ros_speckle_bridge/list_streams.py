#!/usr/bin/env python3
"""
List Streams CLI Utility
Standalone script to list available Speckle streams for the authenticated user
"""

import sys
import os
from tabulate import tabulate
from ros_speckle_bridge.speckle_client import SpeckleClient


def main():
    """List available Speckle streams"""
    # Check for token
    if not os.getenv("SPECKLE_TOKEN"):
        print("ERROR: SPECKLE_TOKEN environment variable not set", file=sys.stderr)
        print("\nPlease set your Speckle Personal Access Token:", file=sys.stderr)
        print("  export SPECKLE_TOKEN='your_token_here'", file=sys.stderr)
        print("\nGet your token from: https://app.speckle.systems/profile", file=sys.stderr)
        return 1
    
    try:
        # Initialize client
        print("Connecting to Speckle...")
        client = SpeckleClient()
        
        # Fetch streams
        print("Fetching streams...\n")
        streams = client.list_streams(limit=50)
        
        if not streams:
            print("No streams found for this account.")
            return 0
        
        # Format as table
        table_data = []
        for stream in streams:
            table_data.append([
                stream.id,
                stream.name,
                stream.description[:50] if stream.description else "",
                getattr(stream, 'updatedDate', 'N/A')
            ])
        
        headers = ["Stream ID", "Name", "Description", "Updated"]
        print(tabulate(table_data, headers=headers, tablefmt="grid"))
        
        print(f"\nTotal: {len(streams)} streams")
        print("\nTo use a stream, set the 'stream_id' parameter in config/params.yaml")
        
        return 0
        
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
