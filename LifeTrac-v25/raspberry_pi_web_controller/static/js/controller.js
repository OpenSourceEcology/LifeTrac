/**
 * LifeTrac v25 Web Controller
 * 
 * Handles joystick input, keyboard controls, and WebSocket communication
 * for controlling the LifeTrac v25 via web browser.
 */

// WebSocket connection
let socket = io();

// Control state
let leftJoystick = null;
let rightJoystick = null;
let currentControl = {
    left_x: 0.0,
    left_y: 0.0,
    right_x: 0.0,
    right_y: 0.0
};

// Keyboard control state
let keyboardControl = {
    left_x: 0.0,
    left_y: 0.0,
    right_x: 0.0,
    right_y: 0.0
};

let activeKeys = new Set();

// Timing
let lastCommandTime = 0;
const COMMAND_INTERVAL = 50; // Send commands every 50ms (20Hz)
let commandInterval = null;

// Debug console
let debugMessages = [];
const MAX_DEBUG_MESSAGES = 50;

/**
 * Initialize the application
 */
function init() {
    initJoysticks();
    initKeyboardControls();
    initEmergencyStop();
    initDebugConsole();
    initSocketIO();
    startCommandLoop();
    
    logDebug('LifeTrac v25 Web Controller initialized');
}

/**
 * Initialize on-screen joysticks using nipplejs
 */
function initJoysticks() {
    // Left joystick - Tank steering
    leftJoystick = nipplejs.create({
        zone: document.getElementById('left-joystick'),
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: '#4CAF50',
        size: 180,
        threshold: 0.1,
        fadeTime: 250
    });

    leftJoystick.on('move', (evt, data) => {
        if (data.direction) {
            // Convert nipplejs output to -1.0 to 1.0 range
            const angle = data.angle.radian;
            const force = Math.min(data.force, 2) / 2; // Normalize to 0-1
            
            currentControl.left_x = Math.cos(angle) * force;
            currentControl.left_y = Math.sin(angle) * force;
            
            updateJoystickDisplay('left', currentControl.left_x, currentControl.left_y);
        }
    });

    leftJoystick.on('end', () => {
        currentControl.left_x = 0.0;
        currentControl.left_y = 0.0;
        updateJoystickDisplay('left', 0, 0);
    });

    // Right joystick - Hydraulics
    rightJoystick = nipplejs.create({
        zone: document.getElementById('right-joystick'),
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: '#2196F3',
        size: 180,
        threshold: 0.1,
        fadeTime: 250
    });

    rightJoystick.on('move', (evt, data) => {
        if (data.direction) {
            const angle = data.angle.radian;
            const force = Math.min(data.force, 2) / 2;
            
            currentControl.right_x = Math.cos(angle) * force;
            currentControl.right_y = Math.sin(angle) * force;
            
            updateJoystickDisplay('right', currentControl.right_x, currentControl.right_y);
        }
    });

    rightJoystick.on('end', () => {
        currentControl.right_x = 0.0;
        currentControl.right_y = 0.0;
        updateJoystickDisplay('right', 0, 0);
    });
}

/**
 * Initialize keyboard controls
 */
function initKeyboardControls() {
    document.addEventListener('keydown', handleKeyDown);
    document.addEventListener('keyup', handleKeyUp);
}

function handleKeyDown(e) {
    // Prevent default for control keys
    if (['w', 'a', 's', 'd', 'i', 'j', 'k', 'l', ' '].includes(e.key.toLowerCase())) {
        e.preventDefault();
    }
    
    // Emergency stop on spacebar
    if (e.key === ' ' && !activeKeys.has('space')) {
        activeKeys.add('space');
        emergencyStop();
        return;
    }
    
    activeKeys.add(e.key.toLowerCase());
    updateKeyboardControl();
}

function handleKeyUp(e) {
    activeKeys.delete(e.key.toLowerCase());
    activeKeys.delete('space'); // Always clear space
    updateKeyboardControl();
}

function updateKeyboardControl() {
    // Tank steering (WASD)
    let left_y = 0.0;
    let left_x = 0.0;
    
    if (activeKeys.has('w')) left_y = 0.7;  // Forward
    if (activeKeys.has('s')) left_y = -0.7; // Backward
    if (activeKeys.has('a')) left_x = -0.5; // Left turn
    if (activeKeys.has('d')) left_x = 0.5;  // Right turn
    
    // Hydraulics (IJKL)
    let right_y = 0.0;
    let right_x = 0.0;
    
    if (activeKeys.has('i')) right_y = 0.7;  // Arms up
    if (activeKeys.has('k')) right_y = -0.7; // Arms down
    if (activeKeys.has('j')) right_x = -0.7; // Bucket down
    if (activeKeys.has('l')) right_x = 0.7;  // Bucket up
    
    // Only update if keyboard input is active
    if (activeKeys.size > 0 && !activeKeys.has('space')) {
        currentControl.left_x = left_x;
        currentControl.left_y = left_y;
        currentControl.right_x = right_x;
        currentControl.right_y = right_y;
        
        updateJoystickDisplay('left', left_x, left_y);
        updateJoystickDisplay('right', right_x, right_y);
    } else if (activeKeys.size === 0) {
        // Reset displays when no keys pressed
        updateJoystickDisplay('left', currentControl.left_x, currentControl.left_y);
        updateJoystickDisplay('right', currentControl.right_x, currentControl.right_y);
    }
}

/**
 * Update joystick value displays
 */
function updateJoystickDisplay(side, x, y) {
    document.getElementById(`${side}-x-value`).textContent = x.toFixed(2);
    document.getElementById(`${side}-y-value`).textContent = y.toFixed(2);
}

/**
 * Initialize emergency stop button
 */
function initEmergencyStop() {
    const stopButton = document.getElementById('emergency-stop');
    
    stopButton.addEventListener('click', emergencyStop);
    
    // Touch support
    stopButton.addEventListener('touchstart', (e) => {
        e.preventDefault();
        emergencyStop();
    });
}

function emergencyStop() {
    logDebug('🛑 EMERGENCY STOP ACTIVATED', 'error');
    
    // Stop all movement
    currentControl.left_x = 0.0;
    currentControl.left_y = 0.0;
    currentControl.right_x = 0.0;
    currentControl.right_y = 0.0;
    
    // Update displays
    updateJoystickDisplay('left', 0, 0);
    updateJoystickDisplay('right', 0, 0);
    
    // Send emergency stop via WebSocket
    socket.emit('emergency_stop');
    
    // Flash button
    const stopButton = document.getElementById('emergency-stop');
    stopButton.style.background = 'linear-gradient(135deg, #ffff00 0%, #ff0000 100%)';
    setTimeout(() => {
        stopButton.style.background = 'linear-gradient(135deg, #ff0000 0%, #cc0000 100%)';
    }, 200);
}

/**
 * Initialize debug console
 */
function initDebugConsole() {
    const debugToggle = document.getElementById('debug-toggle');
    const debugContent = document.getElementById('debug-content');
    
    debugToggle.addEventListener('click', () => {
        debugContent.classList.toggle('hidden');
        debugToggle.textContent = debugContent.classList.contains('hidden') 
            ? 'Debug Console ▼' 
            : 'Debug Console ▲';
    });
}

function logDebug(message, type = 'info') {
    const timestamp = new Date().toLocaleTimeString();
    const logMessage = `[${timestamp}] ${message}`;
    
    debugMessages.push({ message: logMessage, type });
    
    // Keep only last MAX_DEBUG_MESSAGES
    if (debugMessages.length > MAX_DEBUG_MESSAGES) {
        debugMessages.shift();
    }
    
    updateDebugDisplay();
}

function updateDebugDisplay() {
    const debugDiv = document.getElementById('debug-messages');
    debugDiv.innerHTML = debugMessages
        .map(msg => `<div class="${msg.type}">${msg.message}</div>`)
        .join('');
    
    // Auto-scroll to bottom
    debugDiv.scrollTop = debugDiv.scrollHeight;
}

/**
 * Initialize WebSocket connection
 */
function initSocketIO() {
    socket.on('connect', () => {
        logDebug('✓ Connected to server', 'info');
        document.getElementById('mqtt-status').querySelector('span').className = 'status-connected';
        document.getElementById('mqtt-status').querySelector('span').textContent = 'Connected';
        hideConnectionWarning();
    });

    socket.on('disconnect', () => {
        logDebug('✗ Disconnected from server', 'error');
        document.getElementById('mqtt-status').querySelector('span').className = 'status-disconnected';
        document.getElementById('mqtt-status').querySelector('span').textContent = 'Disconnected';
        showConnectionWarning();
    });

    socket.on('connection_response', (data) => {
        logDebug(`Server connection established. MQTT: ${data.mqtt_connected}`);
        if (data.mqtt_connected) {
            document.getElementById('mqtt-status').querySelector('span').className = 'status-connected';
            document.getElementById('mqtt-status').querySelector('span').textContent = 'Connected';
        }
    });

    socket.on('command_sent', (data) => {
        if (data.success) {
            const latencySpan = document.getElementById('latency-value');
            const latency = Date.now() - lastCommandTime;
            latencySpan.textContent = latency;
        }
    });

    socket.on('lifetrac_status', (data) => {
        // Handle status updates from LifeTrac
        logDebug(`Status update: ${JSON.stringify(data)}`);
    });

    socket.on('emergency_stop_confirmed', (data) => {
        logDebug('Emergency stop confirmed by server', 'warning');
    });

    socket.on('error', (data) => {
        logDebug(`Error: ${data.message}`, 'error');
    });
}

/**
 * Start command loop to send control data periodically
 */
function startCommandLoop() {
    commandInterval = setInterval(() => {
        sendControlCommand();
    }, COMMAND_INTERVAL);
}

/**
 * Send control command to server
 */
function sendControlCommand() {
    const commandData = {
        left_x: currentControl.left_x,
        left_y: currentControl.left_y,
        right_x: currentControl.right_x,
        right_y: currentControl.right_y
    };
    
    lastCommandTime = Date.now();
    socket.emit('control_command', commandData);
}

/**
 * Show/hide connection warning
 */
function showConnectionWarning() {
    document.getElementById('connection-warning').classList.remove('hidden');
}

function hideConnectionWarning() {
    document.getElementById('connection-warning').classList.add('hidden');
}

/**
 * Handle page visibility to stop sending commands when tab is hidden
 */
document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
        // Stop all movement when tab is hidden
        emergencyStop();
        logDebug('Tab hidden - emergency stop triggered', 'warning');
    }
});

/**
 * Handle page unload to stop all movement
 */
window.addEventListener('beforeunload', () => {
    emergencyStop();
});

// Initialize when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
