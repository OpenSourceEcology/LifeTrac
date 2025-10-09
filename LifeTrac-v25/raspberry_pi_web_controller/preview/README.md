# Web Controller Preview Assets

This directory contains assets for generating preview screenshots of the LifeTrac v25 web controller interface.

## Files

### camera-placeholder.svg
An SVG placeholder image that simulates a camera feed view. Used in the standalone HTML to provide a realistic preview without requiring an actual camera connection.

### standalone.html
A self-contained HTML page that includes:
- Complete web controller interface styling (embedded CSS)
- Joystick initialization code (embedded JavaScript)
- No Flask backend dependencies
- Uses the camera placeholder image

This page is used by the GitHub Actions workflow to generate screenshots automatically.

### web-controller-preview.png
Auto-generated screenshot of the web controller interface showing:
- Header with status indicators
- Camera feed area with placeholder
- Left joystick (Tank Steering - green)
- Right joystick (Hydraulics - orange)
- Emergency stop button
- Keyboard shortcuts reference

## Usage

### Manual Screenshot Generation

To generate a screenshot locally:

1. Install Playwright:
```bash
npm init -y
npm install --save-dev @playwright/test
npx playwright install --with-deps chromium
```

2. Start a local HTTP server:
```bash
cd preview
python3 -m http.server 8080
```

3. Run the screenshot script:
```bash
node screenshot.js
```

### Automated Screenshot Generation

The GitHub Actions workflow (`.github/workflows/generate-web-controller-preview.yml`) automatically:
- Triggers on changes to web controller files
- Generates a fresh screenshot
- Uploads it as an artifact
- Commits it back to the repository

You can also manually trigger the workflow from the GitHub Actions tab.

## Customization

To customize the preview appearance:
1. Edit `standalone.html` to change the UI layout or styling
2. Edit `camera-placeholder.svg` to change the placeholder image
3. Run the screenshot generation process to create a new preview

## Integration

The preview image is referenced in:
- `LifeTrac-v25/README.md` - Main documentation
- GitHub repository preview for the web controller feature
