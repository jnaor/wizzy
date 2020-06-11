
# Wireless Local Access Point Bridge
## This guide will explain how to create an access point and bridge it with the wired network , on a RaspberryPI 3 and later.

### First install some packages:
> sudo apt install hostapd bridge-utils  

### Configure hostapd as a service:
> sudo systemctl unmask hostapd  
> sudo systemctl enable hostapd  

### Remove network-manager:
> sudo apt purge network-manager

### Create the file /etc/hostapd/hostapd.conf and fill it with the following lines:

> interface=wlan0  
> hw_mode=g  
> bridge=br0  
> country_code=IL  
> ignore_broadcast_ssid=0  
> ieee80211d=1  
> ssid=WIzzyFI  
> channel=6  
> wme_enabled=0  
> macaddr_acl=0  
> auth_algs=1  
> wpa=2  
> wpa_passphrase=wizzywizzy  
> wpa_key_mgmt=WPA-PSK  
> wpa_pairwise=TKIP  
> rsn_pairwise=CCMP  

### Save and exit.
### Edit the hostapd default configuration file /etc/default/hostapd:

### Change:
>  DAEMON_CONF=""  
### To:  
>  DAEMON_CONF="/etc/hostapd/hostapd.conf"


### Add the following lines to /etc/network/interfaces:
>  auto eth0  
>  iface eth0 inet static  
> 		address 0.0.0.0  
> 		netmask 0.0.0.0  
>  
>  auto br0  
>  iface br0 inet static  
> 		address 10.0.0.10  
> 		netmast 255.255.255.0  
> 		bridge_ports eth0  
> 		bridge_stp off  
> 		bridge_fd 0  
> 		bridge_maxwait 0  
> 		post-up /etc/network/if-up.d/hostapd.start

### Save and exit.

### Create the file post-up /etc/network/if-up.d/hostapd.start:
> 		rfkill unblock wlan
> 		ip addr flush dev wlan0
>     ifconfig br0 down  
> 		ifconfig br0 up  
> 		ifconfig wlan0 down  
> 		ifconfig wlan0 up  
> 		/etc/init.d/hostapd stop  
> 		/etc/init.d/hostapd start  
  
### Save and exit.
:+1:
## Done, Lets give it a try

### Reboot the RPi and wait for a new SSID network named WIzzyFI

#### Now it should be possible to connect with a static IP to the WIzzyFI wireless network.
#### Connect with the password wizzywizzy and change the IP address of the wireless adapter on the laptop to 10.0.0.40/24

## Good Luck!
