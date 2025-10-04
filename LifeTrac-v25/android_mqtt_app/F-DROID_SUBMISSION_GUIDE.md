# F-Droid Submission Guide for LifeTrac v25 Remote Control App

This guide provides step-by-step instructions for submitting the LifeTrac v25 Android MQTT Remote Control app to the F-Droid open-source app store.

## About F-Droid

F-Droid is a free and open-source Android app repository that only hosts FOSS (Free and Open Source Software) applications. It's an excellent fit for the LifeTrac project as it:
- Promotes open-source software
- Builds all apps from source for transparency
- Doesn't require Google services
- Is trusted by privacy-conscious users
- Aligns with Open Source Ecology's mission

## Prerequisites

Before submitting to F-Droid, ensure:
- ✅ App is fully open source with compatible license (GPLv3, MIT, Apache 2.0, etc.)
- ✅ Source code is available in a public repository (GitHub)
- ✅ App builds successfully from source
- ✅ No proprietary dependencies or binary blobs
- ✅ No tracking, ads, or anti-features

## Important Note About Kodular

**⚠️ Challenge:** Kodular apps typically cannot be directly submitted to F-Droid because:
1. Kodular uses proprietary build tools
2. F-Droid requires building from source using only open-source tools
3. Kodular-generated APKs include some closed-source components

**Solution:** You have two options:

### Option 1: Convert to Native Android Project (Recommended for F-Droid)
To properly submit to F-Droid, the app should be converted to a native Android project using standard build tools (Android Studio, Gradle).

### Option 2: Use MIT App Inventor with Open-Source Build Process
MIT App Inventor has better open-source tooling and can potentially be built reproducibly.

## Step-by-Step Submission Process

### Phase 1: Prepare the Application

#### 1.1 Convert to Native Android (If Not Already)

If using Kodular/MIT App Inventor, you'll need to either:

**A. Create Native Android Version**
```bash
# Use Android Studio to create a new project
# Package name: org.opensourceecology.lifetrac.v25.remote
# Minimum SDK: API 24 (Android 7.0)
# Target SDK: API 33 (Android 13)
```

**B. Use App Inventor Build Server (Limited F-Droid compatibility)**
```bash
# Export source from MIT App Inventor
# This may not meet F-Droid's reproducible build requirements
```

#### 1.2 Set Up Source Repository

Ensure your GitHub repository has:
```
LifeTrac/
├── LifeTrac-v25/
│   └── android_mqtt_app/
│       ├── app/                    # Android app source (if native)
│       │   ├── src/
│       │   ├── build.gradle
│       │   └── AndroidManifest.xml
│       ├── build.gradle            # Root build file
│       ├── settings.gradle
│       ├── LICENSE                 # Open source license
│       ├── README.md
│       └── metadata/               # F-Droid metadata
│           └── en-US/
│               ├── full_description.txt
│               ├── short_description.txt
│               └── images/
│                   ├── icon.png
│                   ├── phoneScreenshots/
│                   └── sevenInchScreenshots/
```

#### 1.3 Create F-Droid Metadata Directory

```bash
cd /path/to/LifeTrac/LifeTrac-v25/android_mqtt_app
mkdir -p metadata/en-US/images/phoneScreenshots
mkdir -p metadata/en-US/images/sevenInchScreenshots
```

#### 1.4 Create Metadata Files

**metadata/en-US/full_description.txt:**
```
LifeTrac v25 Remote Control provides wireless MQTT-based control for the LifeTrac v25 open-source tractor via WiFi.

FEATURES:
• Dual virtual joysticks for tank steering and hydraulic control
• Real-time MQTT communication at 20Hz frequency
• Emergency stop functionality for immediate safety response
• Visual connection status indicators for WiFi and MQTT
• Configurable MQTT broker settings

TECHNICAL DETAILS:
• Communicates with Arduino Opta controller via MQTT
• Compatible with Mosquitto MQTT broker on Raspberry Pi
• Supports JSON control messages for precise movement
• Implements 1-second safety timeout for fail-safe operation

REQUIREMENTS:
• LifeTrac v25 system with Arduino Opta controller
• Raspberry Pi with Mosquitto MQTT broker configured
• WiFi connectivity to same network as LifeTrac system
• Android 7.0 or newer

This app is part of the Open Source Ecology project, enabling anyone to build and operate industrial machinery using open-source designs and software.

Source code: https://github.com/OpenSourceEcology/LifeTrac/tree/main/LifeTrac-v25/android_mqtt_app
```

**metadata/en-US/short_description.txt:**
```
Wireless MQTT remote control for LifeTrac v25 open-source tractor
```

**metadata/en-US/title.txt:**
```
LifeTrac v25 Remote
```

#### 1.5 Prepare Screenshots

Take screenshots of:
- Main control screen with joysticks
- Settings configuration screen
- Connected status display
- Emergency stop interface

Requirements:
- Format: PNG or JPEG
- Phone: 1080x1920 or similar 16:9 ratio
- Tablet: 1920x1080 or similar 16:10 ratio
- No device frames needed

### Phase 2: Submit Request to F-Droid

#### 2.1 Create F-Droid Account

1. Go to https://gitlab.com/fdroid/fdroiddata
2. Create a GitLab account if you don't have one
3. Fork the fdroiddata repository

#### 2.2 Prepare Application Metadata File

Create a metadata file for your app. F-Droid uses YAML format:

**Template: `metadata/org.opensourceecology.lifetrac.v25.remote.yml`**

```yaml
Categories:
  - Internet
  - System
License: GPL-3.0-or-later
AuthorName: Open Source Ecology
AuthorEmail: info@opensourceecology.org
AuthorWebSite: https://www.opensourceecology.org/
WebSite: https://github.com/OpenSourceEcology/LifeTrac
SourceCode: https://github.com/OpenSourceEcology/LifeTrac
IssueTracker: https://github.com/OpenSourceEcology/LifeTrac/issues
Changelog: https://github.com/OpenSourceEcology/LifeTrac/releases

AutoName: LifeTrac v25 Remote
Summary: Wireless MQTT remote control for LifeTrac v25 tractor

Description: |-
    LifeTrac v25 Remote Control provides wireless MQTT-based control for the
    LifeTrac v25 open-source tractor via WiFi.
    
    Features:
    * Dual virtual joysticks for tank steering and hydraulic control
    * Real-time MQTT communication at 20Hz frequency
    * Emergency stop functionality
    * Visual connection status indicators
    * Configurable MQTT broker settings
    
    This app requires a LifeTrac v25 system with Arduino Opta controller and
    Mosquitto MQTT broker running on a Raspberry Pi.

RepoType: git
Repo: https://github.com/OpenSourceEcology/LifeTrac

Builds:
  - versionName: '1.0'
    versionCode: 1
    commit: v1.0
    subdir: LifeTrac-v25/android_mqtt_app/app
    gradle:
      - yes
    prebuild: echo "sdk.dir=$ANDROID_HOME" > ../local.properties

AutoUpdateMode: Version v%v
UpdateCheckMode: Tags
CurrentVersion: '1.0'
CurrentVersionCode: 1
```

#### 2.3 Submit Merge Request

1. **Fork fdroiddata repository** on GitLab
2. **Clone your fork:**
```bash
git clone https://gitlab.com/YOUR_USERNAME/fdroiddata.git
cd fdroiddata
```

3. **Create a new branch:**
```bash
git checkout -b add-lifetrac-v25-remote
```

4. **Add your metadata file:**
```bash
cp /path/to/org.opensourceecology.lifetrac.v25.remote.yml metadata/
```

5. **Commit and push:**
```bash
git add metadata/org.opensourceecology.lifetrac.v25.remote.yml
git commit -m "New app: LifeTrac v25 Remote Control"
git push origin add-lifetrac-v25-remote
```

6. **Create Merge Request:**
- Go to https://gitlab.com/fdroid/fdroiddata/-/merge_requests
- Click "New merge request"
- Select your branch
- Title: "New app: LifeTrac v25 Remote Control"
- Description: Include app details and why it should be included

### Phase 3: F-Droid Review Process

#### 3.1 What F-Droid Checks

F-Droid reviewers will verify:
- ✅ App builds successfully from source
- ✅ No proprietary libraries or blobs
- ✅ License is compatible with F-Droid
- ✅ Metadata is accurate
- ✅ No tracking or anti-features
- ✅ Follows Android best practices

#### 3.2 Common Issues and Solutions

**Issue: Build Fails**
```bash
# Solution: Test build locally using F-Droid build tools
git clone https://gitlab.com/fdroid/fdroidserver.git
cd fdroidserver
./fdroid build org.opensourceecology.lifetrac.v25.remote
```

**Issue: Proprietary Dependencies**
```
# Solution: Replace with open-source alternatives
# Example: Use Eclipse Paho MQTT instead of proprietary libraries
```

**Issue: Binary Blobs Detected**
```
# Solution: Remove all pre-compiled libraries
# Build everything from source
```

#### 3.3 Respond to Feedback

F-Droid maintainers may request changes:
- Update metadata file as requested
- Fix build issues
- Remove problematic dependencies
- Update documentation

Push changes to your merge request branch:
```bash
git add .
git commit -m "Address review feedback: [description]"
git push origin add-lifetrac-v25-remote
```

### Phase 4: Post-Approval Maintenance

#### 4.1 Update Process

When releasing new versions:

1. **Tag release in GitHub:**
```bash
git tag -a v1.1 -m "Version 1.1 release"
git push origin v1.1
```

2. **Update F-Droid metadata:**
```bash
# F-Droid can auto-update if configured correctly
# Or manually update metadata/org.opensourceecology.lifetrac.v25.remote.yml
```

#### 4.2 Monitor Build Status

Check build status at:
- https://f-droid.org/packages/org.opensourceecology.lifetrac.v25.remote/
- https://gitlab.com/fdroid/fdroiddata/-/pipelines

## Alternative: F-Droid Build Infrastructure

If converting to native Android is not immediately possible, consider:

### Option A: Use IzzyOnDroid Repository

IzzyOnDroid is a third-party F-Droid repository with less strict requirements:
- Accepts APKs built by developers
- Still requires open source license
- Good stepping stone to main F-Droid repo

Submission: https://apt.izzysoft.de/fdroid/

### Option B: Self-Host F-Droid Repository

Create your own F-Droid repository:

```bash
# Install F-Droid server tools
sudo apt install fdroidserver

# Create repository
fdroid init

# Add your APK
cp LifeTracV25_Remote.apk repo/

# Update repository
fdroid update --create-metadata

# Deploy to web server
rsync -av repo/ user@server:/var/www/fdroid/
```

## Checklist Before Submission

- [ ] App is 100% open source
- [ ] Source code is publicly available on GitHub
- [ ] App builds successfully using standard Android tools
- [ ] No proprietary dependencies
- [ ] LICENSE file included in repository
- [ ] Metadata files created (full_description.txt, short_description.txt)
- [ ] Screenshots prepared (phone and tablet sizes)
- [ ] F-Droid metadata YAML file created
- [ ] Test build completed successfully
- [ ] Documentation updated with F-Droid installation instructions

## Resources

- **F-Droid Documentation:** https://f-droid.org/docs/
- **Submission Guide:** https://f-droid.org/docs/Submitting_to_F-Droid/
- **Build Metadata Reference:** https://f-droid.org/docs/Build_Metadata_Reference/
- **fdroiddata Repository:** https://gitlab.com/fdroid/fdroiddata
- **F-Droid Forum:** https://forum.f-droid.org/

## Support

For help with F-Droid submission:
1. Check F-Droid documentation
2. Ask in F-Droid forum
3. Review existing merge requests for examples
4. Contact F-Droid community on Matrix: #fdroid:f-droid.org

---

**Note:** Converting a Kodular/MIT App Inventor app to native Android for F-Droid compliance is a significant undertaking. Consider the trade-offs between ease of development (Kodular) and open-source distribution (native Android for F-Droid). For maximum reach, you may want to:
1. Keep Kodular version for easy development and Google Play distribution
2. Create native Android version specifically for F-Droid
3. Or distribute APK directly from GitHub releases for users who want to sideload