# Plugin System Migration Guide

## Summary of Changes

The ROS Speckle Bridge has been refactored to use a **plugin architecture** for extensibility.

### What Changed

**Before:**
- Monolithic bridge node
- All features compiled into core package
- Hard dependencies on simulation frameworks

**After:**
- Core bridge remains lightweight
- Optional plugins for extended functionality
- Simulation integrations moved to `examples/`

### New Structure

```
ros_bim_stack/
├── ros_speckle_bridge/
│   └── plugins/              # 🆕 Plugin system
│       ├── base.py           # Plugin interface
│       ├── loader.py         # Dynamic plugin loader
│       └── mesh_exporter.py  # Built-in mesh export
│
└── examples/                 # 🆕 Integration examples
    └── gazebo_integration/   # Gazebo Harmonic demo
        ├── ros_speckle_gazebo/
        │   └── gazebo_spawner.py
        ├── config/
        │   └── gazebo_params.yaml
        ├── docker-compose.yml
        └── README.md
```

## Migration Steps

### For Core Bridge Users

**No changes required!** The core bridge works exactly as before.

Your existing `params.yaml` files will continue to work. Plugins are optional.

### For Gazebo Integration Users

If you were using a custom Gazebo integration:

**Before:**
```bash
docker-compose -f docker-compose.sim.yml up
```

**After:**
```bash
cd examples/gazebo_integration
docker-compose up
```

### Adding Plugin Support

To enable plugins in your existing setup:

1. **Update params.yaml:**
```yaml
/**:
  ros__parameters:
    # ... existing config ...
    
    # Add plugin section
    plugins:
      - type: "mesh_exporter"
        enabled: true
        config:
          output_path: "/shared_meshes"
          format: "dae"
```

2. **No code changes needed** - The bridge automatically loads plugins from config.

## Benefits

### For Users
- **Lighter core package**: Only install what you need
- **Faster startup**: Only load enabled plugins
- **Better testing**: Test core bridge without simulators

### For Developers
- **Easier maintenance**: Plugins isolated from core
- **Extensibility**: Add custom integrations without forking
- **Clean dependencies**: Simulation packages not required for core

## Available Plugins

### Built-in
- **mesh_exporter**: Export BIM geometry to OBJ/DAE/STL files

### Examples
- **gazebo_spawner**: Spawn entities in Gazebo Harmonic
- **isaac_spawner**: NVIDIA Isaac Sim integration *(coming soon)*

### Custom
- See [doc/extending.md](extending.md) for creating your own

## Troubleshooting

### "Plugin not found" error

**Problem:**
```
ERROR: Plugin 'my_plugin' not found
```

**Solution:**
Use full path or module name:
```yaml
# File path
- type: "/absolute/path/to/plugin.py:ClassName"

# Module path
- type: "package.module.ClassName"
```

### Plugin initialization fails

**Problem:**
```
ERROR: Plugin initialization failed: ...
```

**Solution:**
1. Check plugin configuration in `params.yaml`
2. Verify all required dependencies are installed
3. Check logs for specific error message

### Plugins not loading

**Problem:**
Plugins configured but not running

**Solution:**
1. Ensure `enabled: true` in config
2. Check plugin returns `True` from `initialize()`
3. Verify no syntax errors in plugin file

## Next Steps

- Review [examples/gazebo_integration](../examples/gazebo_integration/README.md)
- Create custom plugins - see [extending.md](extending.md)
- Check updated [README.md](../README.md) for new architecture

## Questions?

Open an issue on GitHub or check the documentation in `doc/`.
