# lidar_driver
set lidar 192.169.6.6 and 192.168.6.7

# set net bridge br67 add eth0 and eth1

# use netplan
sudo vim /etc/netplan/01-network-manager-all.yaml

# Let NetworkManager manage all devices on this system
network:
  version: 2
  renderer: NetworkManager
  ethernets:
      eth0: {}
      eth1: {}
      usb0: 
        dhcp4: true

  bridges:
      br67:
        interfaces: [eth0, eth1]
        addresses: [192.168.6.10/24]
        gateway4: 192.168.6.1
        routes:
          - to: 0.0.0.0/0
            via: 192.168.6.1
            metric: 30000

# restart netplan
sudo netplan apply

# show bridge
sudo brctl show 
ifconfig br67
