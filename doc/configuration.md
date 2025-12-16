# Configuration Guide

## Authentication

Set Speckle Personal Access Token as environment variable:

```bash
export SPECKLE_TOKEN="your_personal_access_token"
```

Get your token from: https://app.speckle.systems/profile

## Parameters

Edit `ros_speckle_bridge/config/params.yaml`:

```yaml
/**:
  ros__parameters:
    # Speckle server URL
    host: "https://app.speckle.systems"
    
    # Target BIM model (required)
    stream_id: ""
    
    # Commit version ("latest" or specific commit ID)
    commit_id: "latest"
    
    # Coordinate alignment offset [x, y, z]
    # Subtracted from BIM coordinates to align with ROS map origin
    datum: [0.0, 0.0, 0.0]
    
    # ROS frame identifiers
    frame_id: "map"
    bim_frame_id: "bim_origin"
    
    # Category filtering
    filters:
      # Only publish these categories (empty = all)
      allow: []
      # Example: allow: ["Walls", "Floors", "Doors"]
      
      # Never publish these categories
      deny: []
      # Example: deny: ["Furniture", "Plumbing Fixtures"]
```

## List Available Streams

Use the CLI utility to discover your Speckle streams:

```bash
ros2 run ros_speckle_bridge list_streams
```

Output example:
```
Stream ID              Name              Description
─────────────────────────────────────────────────────
a1b2c3d4e5            Office Building    Level 1-3 architectural model
f6g7h8i9j0            Warehouse Site     Structural and MEP
```

Copy the desired `Stream ID` to your `params.yaml`.

## Coordinate System Alignment

### Understanding Datum

The `datum` parameter defines the offset between BIM world coordinates and your ROS map origin.

**Example scenario:**
- BIM model origin: `[1000.0, 2000.0, 0.0]` meters (site survey coordinates)
- Desired ROS map origin: `[0.0, 0.0, 0.0]`
- Set `datum: [1000.0, 2000.0, 0.0]`

This offset is subtracted from all BIM coordinates before the Y-up to Z-up rotation.

### Frame Transform

The bridge publishes a static TF transform:

```
map → bim_origin
```

Translation: `datum` value
Rotation: Identity (handled in converter)
