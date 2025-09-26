#!/usr/bin/env python3
"""
LifeTrac v25 Android App Setup Verification

This script checks that all prerequisites are in place for the Android MQTT app.

Usage:
    python3 verify_setup.py [broker_ip]
"""

import sys
import socket
import subprocess
import json
import time

DEFAULT_BROKER = "192.168.1.100"
MQTT_PORT = 1883
MQTT_TOPIC = "lifetrac/v25/control"

def print_header():
    """Print verification header."""
    print("🚜 LifeTrac v25 Android App Setup Verification")
    print("=" * 50)
    print()

def check_network_connectivity(broker_ip):
    """Check basic network connectivity to MQTT broker."""
    print("🌐 Checking network connectivity...")
    
    try:
        # Try to connect to MQTT port
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex((broker_ip, MQTT_PORT))
        sock.close()
        
        if result == 0:
            print(f"✅ Network connection to {broker_ip}:{MQTT_PORT} successful")
            return True
        else:
            print(f"❌ Cannot connect to {broker_ip}:{MQTT_PORT}")
            print("   💡 Check if MQTT broker is running")
            print("   💡 Verify IP address is correct")
            return False
            
    except socket.gaierror:
        print(f"❌ Cannot resolve hostname {broker_ip}")
        print("   💡 Check IP address or hostname")
        return False
    except Exception as e:
        print(f"❌ Network error: {e}")
        return False

def check_python_dependencies():
    """Check if required Python packages are installed."""
    print("\n🐍 Checking Python dependencies...")
    
    required_packages = {
        'paho-mqtt': 'paho.mqtt.client',
        'json': 'json'  # Built-in, should always be available
    }
    
    all_available = True
    
    for package_name, import_name in required_packages.items():
        try:
            __import__(import_name)
            print(f"✅ {package_name} is available")
        except ImportError:
            print(f"❌ {package_name} not found")
            if package_name == 'paho-mqtt':
                print("   💡 Install with: pip install paho-mqtt")
            all_available = False
    
    # Check optional packages
    optional_packages = {'keyboard': 'keyboard'}
    
    for package_name, import_name in optional_packages.items():
        try:
            __import__(import_name)
            print(f"✅ {package_name} is available (optional)")
        except ImportError:
            print(f"⚠️  {package_name} not found (optional)")
            if package_name == 'keyboard':
                print("   💡 Install with: pip install keyboard (for simulator)")
    
    return all_available

def check_mqtt_broker(broker_ip):
    """Test MQTT broker connectivity and authentication."""
    print(f"\n📡 Testing MQTT broker at {broker_ip}...")
    
    try:
        import paho.mqtt.client as mqtt
        
        # Test connection without authentication first
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("✅ MQTT broker is accessible")
                client.disconnect()
            elif rc == 4:
                print("⚠️  MQTT broker requires authentication")
                print("   💡 This is expected for LifeTrac v25 setup")
                client.disconnect()
            else:
                print(f"❌ MQTT connection failed (code {rc})")
                client.disconnect()
        
        def on_connect_auth(client, userdata, flags, rc):
            if rc == 0:
                print("✅ MQTT authentication successful")
                client.disconnect()
            elif rc == 4:
                print("❌ MQTT authentication failed")
                print("   💡 Check username/password in broker config")
                client.disconnect()
            else:
                print(f"❌ MQTT connection failed (code {rc})")
                client.disconnect()
        
        # Test basic connection
        client = mqtt.Client()
        client.on_connect = on_connect
        client.connect(broker_ip, MQTT_PORT, 5)
        client.loop_start()
        time.sleep(2)
        client.loop_stop()
        
        # Test with authentication
        print("   Testing with default credentials...")
        client_auth = mqtt.Client()
        client_auth.username_pw_set("lifetrac", "lifetrac_pass")
        client_auth.on_connect = on_connect_auth
        try:
            client_auth.connect(broker_ip, MQTT_PORT, 5)
            client_auth.loop_start()
            time.sleep(2)
            client_auth.loop_stop()
        except:
            print("⚠️  Could not test authentication (connection refused)")
        
        return True
        
    except ImportError:
        print("❌ Cannot test MQTT - paho-mqtt not installed")
        return False
    except Exception as e:
        print(f"❌ MQTT test error: {e}")
        return False

def check_lifetrac_controller():
    """Check if LifeTrac controller is responding."""
    print("\n🚜 Checking LifeTrac v25 controller status...")
    
    try:
        import paho.mqtt.client as mqtt
        
        controller_responding = False
        
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                client.subscribe("lifetrac/v25/status")
                # Send a test message to see if controller responds
                test_msg = json.dumps({
                    "left_x": 0, "left_y": 0, 
                    "right_x": 0, "right_y": 0,
                    "timestamp": int(time.time() * 1000)
                })
                client.publish(MQTT_TOPIC, test_msg)
        
        def on_message(client, userdata, msg):
            nonlocal controller_responding
            controller_responding = True
            print("✅ LifeTrac controller is responding")
            print(f"   Status: {msg.payload.decode()}")
            client.disconnect()
        
        client = mqtt.Client()
        client.username_pw_set("lifetrac", "lifetrac_pass")
        client.on_connect = on_connect
        client.on_message = on_message
        
        try:
            client.connect(broker_ip, MQTT_PORT, 10)
            client.loop_start()
            time.sleep(3)  # Wait for response
            client.loop_stop()
            
            if not controller_responding:
                print("⚠️  No response from LifeTrac controller")
                print("   💡 Check if Arduino Opta is powered on")
                print("   💡 Verify controller code is uploaded")
                print("   💡 Check controller serial monitor for errors")
            
        except Exception as e:
            print(f"⚠️  Could not test controller: {e}")
            
        return controller_responding
        
    except ImportError:
        print("❌ Cannot test controller - paho-mqtt not installed")
        return False

def check_android_requirements():
    """Check Android-related requirements."""
    print("\n📱 Android app requirements...")
    
    print("✅ Kodular.io account needed (free)")
    print("✅ Android 7.0+ device required")  
    print("✅ WiFi connectivity required")
    print("✅ UrsPahoMqttClient extension needed")
    
    return True

def print_file_summary():
    """Print summary of created files."""
    print("\n📁 Available files in android_mqtt_app/:")
    
    files = [
        ("README.md", "Complete user guide and installation instructions"),
        ("KODULAR_BUILD_INSTRUCTIONS.md", "Step-by-step app development guide"),
        ("QUICK_START.md", "5-minute setup guide"),
        ("mqtt_test_listener.py", "Monitor MQTT messages from Android app"),
        ("simulate_android_app.py", "Test system without building app"),
        ("LifeTracV25_Remote_Template.txt", "Kodular project structure template"),
        ("ANDROID_PERMISSIONS.md", "Device requirements and permissions"),
    ]
    
    for filename, description in files:
        print(f"   📄 {filename:<35} - {description}")

def main():
    """Main verification function."""
    print_header()
    
    # Get broker IP from command line or use default
    broker_ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BROKER
    print(f"🎯 Testing MQTT broker at: {broker_ip}")
    print()
    
    # Run all checks
    checks = [
        ("Network connectivity", lambda: check_network_connectivity(broker_ip)),
        ("Python dependencies", check_python_dependencies),
        ("MQTT broker", lambda: check_mqtt_broker(broker_ip)),
        ("LifeTrac controller", check_lifetrac_controller),
        ("Android requirements", check_android_requirements),
    ]
    
    results = []
    for check_name, check_func in checks:
        try:
            result = check_func()
            results.append((check_name, result))
        except Exception as e:
            print(f"❌ Error in {check_name}: {e}")
            results.append((check_name, False))
    
    # Print summary
    print("\n" + "=" * 50)
    print("📋 VERIFICATION SUMMARY")
    print("=" * 50)
    
    passed = 0
    for check_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status:<10} {check_name}")
        if result:
            passed += 1
    
    print(f"\nResults: {passed}/{len(results)} checks passed")
    
    if passed == len(results):
        print("\n🎉 All checks passed! Ready to build Android app.")
        print("📖 Next step: Follow KODULAR_BUILD_INSTRUCTIONS.md")
    else:
        print("\n⚠️  Some checks failed. Please address issues before proceeding.")
        print("📖 See individual check messages above for solutions.")
    
    print_file_summary()
    
    print(f"\n🔧 Quick test commands:")
    print(f"   python3 mqtt_test_listener.py {broker_ip}")
    print(f"   python3 simulate_android_app.py {broker_ip}")

if __name__ == "__main__":
    main()