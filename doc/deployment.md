# Deployment Guide

## Docker Compose

### Setup

```bash
cd ros_speckle_bridge
cp .env.template .env
nano .env  # Set SPECKLE_TOKEN
```

### Build and Run

```bash
docker-compose up --build
```

### Run in Background

```bash
docker-compose up -d
```

### View Logs

```bash
docker-compose logs -f
```

### Stop

```bash
docker-compose down
```

### Update Configuration

Edit `config/params.yaml`, then restart:

```bash
docker-compose restart
```

## Docker Manual

### Build Image

```bash
cd ros_bim_stack
docker build -f ros_speckle_bridge/Dockerfile -t ros-speckle-bridge:latest .
```

### Run Container

```bash
docker run -d \
  --name speckle_bridge \
  --network host \
  -e SPECKLE_TOKEN="your_token" \
  -v speckle_cache:/root/.ros/speckle_cache \
  ros-speckle-bridge:latest
```

### View Logs

```bash
docker logs -f speckle_bridge
```

### Stop Container

```bash
docker stop speckle_bridge
docker rm speckle_bridge
```

## Systemd Service

### Create Service File

Create `/etc/systemd/system/ros-speckle-bridge.service`:

```ini
[Unit]
Description=ROS Speckle Bridge
After=network.target

[Service]
Type=simple
User=ros
Environment="SPECKLE_TOKEN=your_token_here"
WorkingDirectory=/home/ros/ros2_ws
ExecStart=/bin/bash -c "source install/setup.bash && ros2 launch ros_speckle_bridge bridge.launch.py"
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### Enable and Start

```bash
sudo systemctl daemon-reload
sudo systemctl enable ros-speckle-bridge
sudo systemctl start ros-speckle-bridge
```

### Check Status

```bash
sudo systemctl status ros-speckle-bridge
```

### View Logs

```bash
sudo journalctl -u ros-speckle-bridge -f
```

### Stop and Disable

```bash
sudo systemctl stop ros-speckle-bridge
sudo systemctl disable ros-speckle-bridge
```

## Kubernetes

### ConfigMap

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: speckle-bridge-config
data:
  params.yaml: |
    /**:
      ros__parameters:
        host: "https://app.speckle.systems"
        stream_id: "your_stream_id"
        commit_id: "latest"
        datum: [0.0, 0.0, 0.0]
        filters:
          allow: []
          deny: []
```

### Secret

```yaml
apiVersion: v1
kind: Secret
metadata:
  name: speckle-token
type: Opaque
stringData:
  token: "your_speckle_token_here"
```

### Deployment

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: ros-speckle-bridge
spec:
  replicas: 1
  selector:
    matchLabels:
      app: speckle-bridge
  template:
    metadata:
      labels:
        app: speckle-bridge
    spec:
      containers:
      - name: bridge
        image: ros-speckle-bridge:latest
        env:
        - name: SPECKLE_TOKEN
          valueFrom:
            secretKeyRef:
              name: speckle-token
              key: token
        - name: ROS_DOMAIN_ID
          value: "0"
        volumeMounts:
        - name: config
          mountPath: /workspace/install/ros_speckle_bridge/share/ros_speckle_bridge/config/params.yaml
          subPath: params.yaml
        - name: cache
          mountPath: /root/.ros/speckle_cache
      volumes:
      - name: config
        configMap:
          name: speckle-bridge-config
      - name: cache
        persistentVolumeClaim:
          claimName: speckle-cache-pvc
```

### Apply

```bash
kubectl apply -f configmap.yaml
kubectl apply -f secret.yaml
kubectl apply -f deployment.yaml
```

## Production Considerations

### Security

- Store `SPECKLE_TOKEN` in secrets management (Vault, AWS Secrets Manager)
- Use read-only filesystem where possible
- Run as non-root user in container
- Enable network policies in Kubernetes

### Monitoring

#### Health Checks

Monitor node status:
```bash
ros2 node list | grep speckle_bridge
```

Check topic publishing:
```bash
timeout 5 ros2 topic echo /bim/objects --once
```

#### Prometheus Metrics

Add metrics exporter to bridge node for:
- Objects published count
- Cache hit/miss ratio
- API call latency
- Error rates

#### Logging

Configure structured logging:
```python
import logging
logging.basicConfig(
    format='%(asctime)s [%(name)s] [%(levelname)s] %(message)s',
    level=logging.INFO
)
```

Aggregate logs to centralized system (ELK, Loki)

### Performance

#### Resource Limits

Docker:
```yaml
services:
  speckle_bridge:
    deploy:
      resources:
        limits:
          cpus: '2.0'
          memory: 4G
        reservations:
          cpus: '1.0'
          memory: 2G
```

Kubernetes:
```yaml
resources:
  limits:
    cpu: "2000m"
    memory: "4Gi"
  requests:
    cpu: "1000m"
    memory: "2Gi"
```

#### Cache Persistence

Ensure cache volume survives restarts:
- Docker: Named volume
- Kubernetes: PersistentVolumeClaim
- Systemd: User home directory

### High Availability

For critical deployments:

1. Run multiple instances with different `ROS_DOMAIN_ID`
2. Use load balancer for Speckle API calls
3. Configure automatic restart on failure
4. Set up monitoring alerts
5. Maintain cache backup for disaster recovery

### Updates

#### Rolling Updates

Kubernetes:
```yaml
spec:
  strategy:
    type: RollingUpdate
    rollingUpdate:
      maxSurge: 1
      maxUnavailable: 0
```

Docker Compose:
```bash
docker-compose pull
docker-compose up -d --no-deps --build speckle_bridge
```

#### Configuration Changes

1. Update ConfigMap/params.yaml
2. Restart service
3. Verify using health checks
4. Monitor logs for errors
