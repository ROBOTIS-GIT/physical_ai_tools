# Network Configuration Guide

Complete guide for configuring network communication between DDS and ROS 2 systems.

## Quick Reference

| Scenario | Configuration | Notes |
|----------|---------------|-------|
| **Same Computer** | `domain_id=30` | Simplest setup |
| **Same LAN** | `peers=["192.168.1.100:7400"]`<br>`allow_multicast=True` | Local network |
| **Different Subnets** | `peers=["10.0.1.100:7400"]`<br>`allow_multicast=False` | Requires routing |
| **Internet/Cloud** | `peers=["cloud.example.com:7400"]`<br>`allow_multicast=False` | Public IP or VPN |

## Basic Configuration

### Same Computer

```python
from robotis_dds_python.tools.dds_node import DDSNode

# DDS node
node = DDSNode(name='my_node', domain_id=30)

# ROS 2 (in terminal)
export ROS_DOMAIN_ID=30
ros2 topic list
```

**Key Point**: Just match the Domain ID!

### Same Local Network (LAN)

```python
node = DDSNode(
    name='robot',
    domain_id=30,
    peers=["192.168.1.100:7400"],  # ROS 2 computer IP
    allow_multicast=True,
    network_interface='auto'
)
```

**ROS 2 Side:**
```bash
export ROS_DOMAIN_ID=30
ros2 topic list  # Should see DDS topics
```

### Different Networks (WAN/Internet)

```python
node = DDSNode(
    name='cloud_client',
    domain_id=30,
    peers=["52.12.34.56:7400", "cloud.example.com:7400"],
    allow_multicast=False,  # Required for WAN
    network_interface='auto'
)
```

## Network Parameters

### domain_id
- **Range**: 0-232
- **Default**: 0
- **Must Match**: Between DDS and ROS 2
- **Example**: `domain_id=30`

### peers
- **Format**: `["IP:PORT", "HOSTNAME:PORT"]`
- **Default Port**: 7400
- **Multiple Peers**: List of addresses
- **Examples**:
  ```python
  peers=["192.168.1.100:7400"]
  peers=["robot1.local:7400", "192.168.1.50:7400"]
  peers=["cloud.example.com:7400"]
  ```

### allow_multicast
- **True**: LAN (same subnet) - Auto-discovery
- **False**: WAN/Internet - Peer-to-peer only
- **Default**: True

### network_interface
- **"auto"**: Auto-detect (recommended)
- **"0.0.0.0"**: Listen on all interfaces
- **"192.168.1.100"**: Specific interface IP
- **Example**: `network_interface="192.168.50.100"`

### port_base
- **Default**: 7400
- **Range**: 7400-7410 (DDS standard)
- **Custom**: `port_base=7500`

## Firewall Configuration

### Ubuntu/Debian
```bash
# Allow DDS ports
sudo ufw allow 7400:7410/udp
sudo ufw reload
```

### CentOS/RHEL
```bash
sudo firewall-cmd --add-port=7400-7410/udp --permanent
sudo firewall-cmd --reload
```

### Docker
```yaml
# docker-compose.yml
services:
  ros2:
    network_mode: host  # Recommended
    # or
    ports:
      - "7400-7410:7400-7410/udp"
```

## Complete Examples

### Example 1: Robot on LAN

**Network:**
- Robot: 192.168.1.50
- Cloud: 192.168.1.100
- Same subnet

**Robot Side:**
```python
node = DDSNode(
    name='robot',
    peers=["192.168.1.100:7400"],
    allow_multicast=True
)
```

**Cloud Side (ROS 2):**
```bash
export ROS_DOMAIN_ID=0
ros2 run my_package server
```

### Example 2: Cloud Server (Internet)

**Network:**
- Robot: Behind NAT (private IP)
- Cloud: Public IP 52.12.34.56
- VPN: Not available

**Cloud Side:**
```python
node = DDSNode(
    name='cloud_server',
    network_interface='0.0.0.0',  # Listen on all interfaces
    allow_multicast=False
)
```

**Robot Side:**
```python
node = DDSNode(
    name='robot',
    peers=["52.12.34.56:7400"],
    allow_multicast=False
)
```

**Port Forwarding on Cloud:**
```bash
# External 7400-7410 → Internal 7400-7410
```

### Example 3: Multiple Robots

```python
# Cloud server connecting to 3 robots
node = DDSNode(
    name='cloud',
    peers=[
        "robot1.local:7400",
        "192.168.1.51:7400",
        "192.168.1.52:7400"
    ],
    allow_multicast=True
)
```

## Troubleshooting

### Can't see topics/services

1. **Check Domain ID**
   ```bash
   # ROS 2
   echo $ROS_DOMAIN_ID
   
   # DDS - check domain_id parameter
   ```

2. **Check Network Connectivity**
   ```bash
   ping 192.168.1.100
   ```

3. **Check Firewall**
   ```bash
   # Test if port is open
   nc -u -z 192.168.1.100 7400
   ```

4. **Check Multicast Setting**
   - Same subnet → `allow_multicast=True`
   - Different subnets/Internet → `allow_multicast=False`

### High Latency

1. **Network Route**
   ```bash
   traceroute 192.168.1.100
   ```

2. **Use Direct Connection**
   - Avoid VPN if possible for local networks
   - Use wired connection over WiFi

3. **Optimize QoS**
   - Use `BestEffort` reliability for real-time data
   - Reduce `KeepLast` buffer size

### Connection Drops

1. **Check Network Stability**
   ```bash
   ping -c 100 192.168.1.100
   ```

2. **Increase Timeouts**
   ```python
   response = client.call(request, timeout=10.0)
   ```

3. **Use Reliable QoS**
   ```python
   qos = Qos(Policy.Reliability.Reliable(duration()))
   ```

## Advanced Topics

### NAT Traversal

**Problem**: Robot behind NAT can't receive incoming connections

**Solutions:**
1. **VPN** (Recommended)
   - Use WireGuard, OpenVPN, or Tailscale
   - Creates virtual LAN
   
2. **Port Forwarding**
   - Forward 7400-7410/UDP on router
   - Use Dynamic DNS for changing IPs

3. **Reverse Connection**
   - Cloud initiates connection
   - Robot connects to cloud

### VPN Setup (WireGuard Example)

**Cloud Server:**
```bash
# Install WireGuard
sudo apt install wireguard

# Generate keys
wg genkey | tee privatekey | wg pubkey > publickey

# Configure /etc/wireguard/wg0.conf
[Interface]
Address = 10.0.0.1/24
PrivateKey = <cloud_private_key>
ListenPort = 51820

[Peer]
PublicKey = <robot_public_key>
AllowedIPs = 10.0.0.2/32
```

**Robot:**
```bash
# Configure /etc/wireguard/wg0.conf
[Interface]
Address = 10.0.0.2/24
PrivateKey = <robot_private_key>

[Peer]
PublicKey = <cloud_public_key>
Endpoint = 52.12.34.56:51820
AllowedIPs = 10.0.0.0/24
PersistentKeepalive = 25
```

**Start VPN:**
```bash
sudo wg-quick up wg0
```

**Use VPN IPs in DDS:**
```python
# Cloud
node = DDSNode(peers=["10.0.0.2:7400"])  # Robot VPN IP

# Robot
node = DDSNode(peers=["10.0.0.1:7400"])  # Cloud VPN IP
```

## Best Practices

1. **Use VPN for Internet**: More secure and reliable
2. **Match Domain IDs**: Always verify both sides match
3. **Open Firewall Ports**: UDP 7400-7410
4. **Use Static IPs**: Or DNS names for reliability
5. **Test Locally First**: Verify before deploying remotely
6. **Monitor Network**: Use `ros2 topic hz` to check message rates
