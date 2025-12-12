// Multi-Section IMU Visualizer
let sections = [];
let sectionCounter = 0;

// Right hand colors
const colorsRight = {
    x: [255, 100, 100],
    y: [100, 200, 200],
    z: [255, 230, 100]
};

// Left hand colors
const colorsLeft = {
    x: [255, 100, 200],
    y: [100, 255, 200],
    z: [200, 160, 255]
};

// Data format: timestamp, orientX, orientY, orientZ, accelX, accelY, accelZ, speed, sys, gyro, accel, mag
const defaultData = [
    {"timestamp": "00:00:02.349", "orientX": 356.81, "orientY": 26.94, "orientZ": -77.00, "accelX": 1.02, "accelY": -0.93, "accelZ": 0.61, "speed": 0.24, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:02.460", "orientX": 356.06, "orientY": 26.56, "orientZ": -79.94, "accelX": -0.57, "accelY": -1.26, "accelZ": 0.64, "speed": 0.35, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:02.571", "orientX": 354.94, "orientY": 26.31, "orientZ": -85.00, "accelX": -0.47, "accelY": -0.92, "accelZ": 0.41, "speed": 0.45, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:02.681", "orientX": 354.44, "orientY": 26.06, "orientZ": -85.69, "accelX": -0.17, "accelY": -1.03, "accelZ": 0.65, "speed": 0.59, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:02.791", "orientX": 354.00, "orientY": 25.81, "orientZ": -87.37, "accelX": -0.09, "accelY": 0.67, "accelZ": 0.48, "speed": 0.56, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:02.903", "orientX": 353.69, "orientY": 25.19, "orientZ": -89.25, "accelX": 0.27, "accelY": -0.21, "accelZ": 0.51, "speed": 0.62, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:03.013", "orientX": 352.31, "orientY": 24.56, "orientZ": -90.44, "accelX": 0.91, "accelY": -1.54, "accelZ": 0.74, "speed": 0.81, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:03.124", "orientX": 350.75, "orientY": 24.25, "orientZ": -91.56, "accelX": -0.51, "accelY": -0.05, "accelZ": 0.60, "speed": 0.85, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:03.234", "orientX": 350.37, "orientY": 24.12, "orientZ": -92.31, "accelX": 0.24, "accelY": -0.54, "accelZ": 0.51, "speed": 0.93, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:03.344", "orientX": 349.69, "orientY": 24.00, "orientZ": -93.81, "accelX": 1.38, "accelY": -0.66, "accelZ": 0.52, "speed": 1.05, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:03.456", "orientX": 349.06, "orientY": 23.44, "orientZ": -93.87, "accelX": 0.36, "accelY": -0.45, "accelZ": 0.59, "speed": 1.14, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:03.567", "orientX": 348.06, "orientY": 22.81, "orientZ": -93.75, "accelX": -0.18, "accelY": 0.19, "accelZ": 0.47, "speed": 1.16, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:03.678", "orientX": 347.81, "orientY": 22.69, "orientZ": -91.19, "accelX": 0.33, "accelY": 1.52, "accelZ": 0.12, "speed": 1.07, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:03.788", "orientX": 348.37, "orientY": 22.87, "orientZ": -86.87, "accelX": -0.18, "accelY": 0.03, "accelZ": 0.28, "speed": 1.09, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:03.899", "orientX": 348.75, "orientY": 24.00, "orientZ": -82.50, "accelX": 0.39, "accelY": -1.24, "accelZ": 0.46, "speed": 1.22, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:04.009", "orientX": 349.62, "orientY": 24.44, "orientZ": -79.87, "accelX": -0.33, "accelY": -1.06, "accelZ": 0.65, "speed": 1.33, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:04.120", "orientX": 349.06, "orientY": 26.44, "orientZ": -77.56, "accelX": 0.48, "accelY": -1.03, "accelZ": 0.60, "speed": 1.47, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:04.231", "orientX": 348.31, "orientY": 26.87, "orientZ": -77.94, "accelX": 0.00, "accelY": 0.19, "accelZ": 0.32, "speed": 1.48, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:04.341", "orientX": 348.19, "orientY": 27.37, "orientZ": -78.37, "accelX": -0.19, "accelY": -0.32, "accelZ": 0.46, "speed": 1.54, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:04.451", "orientX": 347.87, "orientY": 28.00, "orientZ": -78.50, "accelX": 0.32, "accelY": -0.96, "accelZ": 0.50, "speed": 1.65, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:04.563", "orientX": 347.75, "orientY": 28.25, "orientZ": -78.00, "accelX": 0.36, "accelY": -0.28, "accelZ": 0.42, "speed": 1.72, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:04.674", "orientX": 347.44, "orientY": 28.44, "orientZ": -77.87, "accelX": -0.50, "accelY": -0.63, "accelZ": 0.57, "speed": 1.80, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:04.785", "orientX": 347.12, "orientY": 28.94, "orientZ": -78.06, "accelX": 0.00, "accelY": -0.69, "accelZ": 0.49, "speed": 1.89, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:04.895", "orientX": 347.00, "orientY": 29.00, "orientZ": -78.37, "accelX": 0.20, "accelY": -0.90, "accelZ": 0.78, "speed": 2.02, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:05.006", "orientX": 346.87, "orientY": 29.06, "orientZ": -78.56, "accelX": -0.33, "accelY": -0.46, "accelZ": 0.43, "speed": 2.08, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0},
    {"timestamp": "00:00:05.117", "orientX": 346.69, "orientY": 29.06, "orientZ": -79.19, "accelX": -0.05, "accelY": -0.44, "accelZ": 0.42, "speed": 2.15, "calSys": 0, "calGyro": 0, "calAccel": 0, "calMag": 0}
];

// For backward compatibility, map orientation to x, y, z
function getDisplayData(d) {
    return {
        x: d.orientX,
        y: d.orientY,
        z: d.orientZ
    };
}

function setup() {
    noCanvas();
    initExistingSections();
    setupAddButton();
}

// Convert timestamp string (00:00:00.000) to milliseconds
function timestampToMs(timestamp) {
    let parts = timestamp.split(':');
    let hours = parseInt(parts[0]);
    let minutes = parseInt(parts[1]);
    let secParts = parts[2].split('.');
    let seconds = parseInt(secParts[0]);
    let ms = parseInt(secParts[1]);
    return (hours * 3600 + minutes * 60 + seconds) * 1000 + ms;
}

function initExistingSections() {
    document.querySelectorAll('.viz-section').forEach(el => {
        let id = parseInt(el.dataset.sectionId);
        createSectionInstance(el, id);
        sectionCounter = Math.max(sectionCounter, id + 1);
    });
}

function createSectionInstance(element, id) {
    let container = element.querySelector('.canvas-container');
    let p5Instance = new p5(sketch => sectionSketch(sketch, id), container);
    
    let section = {
        id: id,
        element: element,
        p5: p5Instance,
        dataRight: JSON.parse(JSON.stringify(defaultData)),
        dataLeft: [],
        currentIndex: 0,
        isPlaying: false,
        currentBrightnessRight: 0,
        targetBrightnessRight: 0,
        currentBrightnessLeft: 0,
        targetBrightnessLeft: 0,
        brightnessHistoryRight: [],
        brightnessHistoryLeft: [],
        currentPage: 0, // 0: main, 1: heatmap, 2: magnitude
        heatmapInstances: {},
        playStartTime: 0,
        dataStartTime: 0,
        magPlaySpeed: 1.0, // Magnitude view playback speed
        magIsPlaying: false // Magnitude view playing state
    };
    
    sections.push(section);
    bindSectionEvents(section);
    updateSectionDisplay(section);
}

function sectionSketch(p, sectionId) {
    p.setup = function() {
        p.createCanvas(680, 380, p.WEBGL);
    };
    
    p.draw = function() {
        let section = sections.find(s => s.id === sectionId);
        if (!section || section.dataRight.length === 0) return;
        
        // Smooth fade brightness for both hands
        section.currentBrightnessRight = p.lerp(section.currentBrightnessRight, section.targetBrightnessRight, 0.08);
        section.targetBrightnessRight *= 0.85;
        section.currentBrightnessLeft = p.lerp(section.currentBrightnessLeft, section.targetBrightnessLeft, 0.08);
        section.targetBrightnessLeft *= 0.85;
        
        // Record brightness history (every 2 frames)
        if (p.frameCount % 2 === 0) {
            section.brightnessHistoryRight.push(section.currentBrightnessRight);
            section.brightnessHistoryLeft.push(section.currentBrightnessLeft);
            if (section.brightnessHistoryRight.length > 400) {
                section.brightnessHistoryRight.shift();
                section.brightnessHistoryLeft.shift();
            }
        }
        
        // Update section element background color (average of both)
        let brightness = Math.floor((section.currentBrightnessRight + section.currentBrightnessLeft) * 0.3);
        section.element.style.backgroundColor = `rgb(${brightness}, ${brightness}, ${brightness})`;
        
        // Canvas background
        p.background(0);
        
        // Timestamp-based playback
        if (section.isPlaying) {
            let currentTime = Date.now();
            let elapsedTime = currentTime - section.playStartTime;
            let dataStartMs = timestampToMs(section.dataRight[0].timestamp);
            let targetTimeMs = dataStartMs + elapsedTime;
            
            // Find the frame that matches current playback time
            let newIndex = section.currentIndex;
            let maxLen = Math.max(section.dataRight.length, section.dataLeft.length);
            for (let i = section.currentIndex; i < maxLen; i++) {
                let frameTimeMs = timestampToMs(section.dataRight[Math.min(i, section.dataRight.length - 1)].timestamp);
                if (frameTimeMs <= targetTimeMs) {
                    newIndex = i;
                } else {
                    break;
                }
            }
            
            // Update if frame changed
            if (newIndex !== section.currentIndex) {
                let prevIndex = section.currentIndex;
                section.currentIndex = newIndex;
                updateMovementBrightness(section, prevIndex);
                updateSectionDisplay(section);
            }
            
            // Loop back to start when finished
            if (section.currentIndex >= maxLen - 1) {
                section.currentIndex = 0;
                section.playStartTime = Date.now();
            }
        }
        
        p.push();
        p.translate(-p.width/2, -p.height/2);
        drawGraph(p, section);
        p.pop();
        
        // Right hand 3D
        p.push();
        p.translate(60, -50, 0);
        draw3D(p, section, 'right');
        p.pop();
        
        // Left hand 3D
        p.push();
        p.translate(220, -50, 0);
        draw3D(p, section, 'left');
        p.pop();
        
        p.push();
        p.translate(-p.width/2, -p.height/2);
        drawBrightnessGraph(p, section);
        p.pop();
    };
}

function updateMovementBrightness(section, prevIndex) {
    // Right hand
    let prevR = section.dataRight[Math.min(prevIndex, section.dataRight.length - 1)];
    let currR = section.dataRight[Math.min(section.currentIndex, section.dataRight.length - 1)];
    
    let dxR = Math.abs(currR.orientX - prevR.orientX);
    let dyR = Math.abs(currR.orientY - prevR.orientY);
    let dzR = Math.abs(currR.orientZ - prevR.orientZ);
    let totalChangeR = dxR + dyR + dzR;
    
    // Left hand
    let totalChangeL = 0;
    if (section.dataLeft.length > 0) {
        let prevL = section.dataLeft[Math.min(prevIndex, section.dataLeft.length - 1)];
        let currL = section.dataLeft[Math.min(section.currentIndex, section.dataLeft.length - 1)];
        
        let dxL = Math.abs(currL.orientX - prevL.orientX);
        let dyL = Math.abs(currL.orientY - prevL.orientY);
        let dzL = Math.abs(currL.orientZ - prevL.orientZ);
        totalChangeL = dxL + dyL + dzL;
    }
    
    // Ignore small changes (noise/tremor threshold)
    let threshold = 15;
    if (totalChangeR > threshold) {
        section.targetBrightnessRight = Math.min(255, section.targetBrightnessRight + (totalChangeR - threshold) * 15);
    }
    if (totalChangeL > threshold) {
        section.targetBrightnessLeft = Math.min(255, section.targetBrightnessLeft + (totalChangeL - threshold) * 15);
    }
}

// For magnitude view: calculate brightness for a specific frame transition
function updateMovementBrightnessForFrame(section, fromIndex, toIndex) {
    // Right hand
    let prevR = section.dataRight[Math.min(fromIndex, section.dataRight.length - 1)];
    let currR = section.dataRight[Math.min(toIndex, section.dataRight.length - 1)];
    
    let dxR = Math.abs(currR.orientX - prevR.orientX);
    let dyR = Math.abs(currR.orientY - prevR.orientY);
    let dzR = Math.abs(currR.orientZ - prevR.orientZ);
    let totalChangeR = dxR + dyR + dzR;
    
    // Left hand
    let totalChangeL = 0;
    if (section.dataLeft.length > 0) {
        let prevL = section.dataLeft[Math.min(fromIndex, section.dataLeft.length - 1)];
        let currL = section.dataLeft[Math.min(toIndex, section.dataLeft.length - 1)];
        
        let dxL = Math.abs(currL.orientX - prevL.orientX);
        let dyL = Math.abs(currL.orientY - prevL.orientY);
        let dzL = Math.abs(currL.orientZ - prevL.orientZ);
        totalChangeL = dxL + dyL + dzL;
    }
    
    // Ignore small changes (noise/tremor threshold)
    let threshold = 5;
    if (totalChangeR > threshold) {
        section.targetBrightnessRight = Math.min(255, section.targetBrightnessRight + (totalChangeR - threshold) * 15);
    }
    if (totalChangeL > threshold) {
        section.targetBrightnessLeft = Math.min(255, section.targetBrightnessLeft + (totalChangeL - threshold) * 15);
    }
}

function drawGraph(p, section) {
    let gx = 15;
    let gy = 15;
    let gw = 350;
    let gh = 170;
    
    p.fill(40);
    p.noStroke();
    p.rect(gx, gy, gw, gh);
    
    p.stroke(60);
    p.strokeWeight(1);
    for (let i = 0; i <= 4; i++) {
        let y = gy + (gh / 4) * i;
        p.line(gx, y, gx + gw, y);
    }
    
    // Right hand lines (solid)
    let spacingR = gw / (section.dataRight.length - 1);
    
    p.stroke(colorsRight.x[0], colorsRight.x[1], colorsRight.x[2]);
    p.strokeWeight(1.5);
    p.noFill();
    p.beginShape();
    for (let i = 0; i < section.dataRight.length; i++) {
        p.vertex(gx + i * spacingR, gy + p.map(section.dataRight[i].orientX, 360, 0, 0, gh));
    }
    p.endShape();
    
    p.stroke(colorsRight.y[0], colorsRight.y[1], colorsRight.y[2]);
    p.beginShape();
    for (let i = 0; i < section.dataRight.length; i++) {
        p.vertex(gx + i * spacingR, gy + p.map(section.dataRight[i].orientY, 90, -90, 0, gh));
    }
    p.endShape();
    
    p.stroke(colorsRight.z[0], colorsRight.z[1], colorsRight.z[2]);
    p.beginShape();
    for (let i = 0; i < section.dataRight.length; i++) {
        p.vertex(gx + i * spacingR, gy + p.map(section.dataRight[i].orientZ, 90, -90, 0, gh));
    }
    p.endShape();
    
    // Left hand lines (dashed style - lighter stroke)
    if (section.dataLeft.length > 0) {
        let spacingL = gw / (section.dataLeft.length - 1);
        
        p.stroke(colorsLeft.x[0], colorsLeft.x[1], colorsLeft.x[2]);
        p.strokeWeight(1);
        p.beginShape();
        for (let i = 0; i < section.dataLeft.length; i++) {
            p.vertex(gx + i * spacingL, gy + p.map(section.dataLeft[i].orientX, 360, 0, 0, gh));
        }
        p.endShape();
        
        p.stroke(colorsLeft.y[0], colorsLeft.y[1], colorsLeft.y[2]);
        p.beginShape();
        for (let i = 0; i < section.dataLeft.length; i++) {
            p.vertex(gx + i * spacingL, gy + p.map(section.dataLeft[i].orientY, 90, -90, 0, gh));
        }
        p.endShape();
        
        p.stroke(colorsLeft.z[0], colorsLeft.z[1], colorsLeft.z[2]);
        p.beginShape();
        for (let i = 0; i < section.dataLeft.length; i++) {
            p.vertex(gx + i * spacingL, gy + p.map(section.dataLeft[i].orientZ, 90, -90, 0, gh));
        }
        p.endShape();
    }
    
    // Current position marker
    let mx = gx + section.currentIndex * spacingR;
    p.stroke(255);
    p.strokeWeight(1);
    p.line(mx, gy, mx, gy + gh);
    
    // Right hand dots
    let dR = section.dataRight[Math.min(section.currentIndex, section.dataRight.length - 1)];
    p.noStroke();
    p.fill(colorsRight.x[0], colorsRight.x[1], colorsRight.x[2]);
    p.ellipse(mx, gy + p.map(dR.orientX, 360, 0, 0, gh), 6, 6);
    p.fill(colorsRight.y[0], colorsRight.y[1], colorsRight.y[2]);
    p.ellipse(mx, gy + p.map(dR.orientY, 90, -90, 0, gh), 6, 6);
    p.fill(colorsRight.z[0], colorsRight.z[1], colorsRight.z[2]);
    p.ellipse(mx, gy + p.map(dR.orientZ, 90, -90, 0, gh), 6, 6);
    
    // Left hand dots (smaller)
    if (section.dataLeft.length > 0) {
        let dL = section.dataLeft[Math.min(section.currentIndex, section.dataLeft.length - 1)];
        p.fill(colorsLeft.x[0], colorsLeft.x[1], colorsLeft.x[2]);
        p.ellipse(mx, gy + p.map(dL.orientX, 360, 0, 0, gh), 4, 4);
        p.fill(colorsLeft.y[0], colorsLeft.y[1], colorsLeft.y[2]);
        p.ellipse(mx, gy + p.map(dL.orientY, 90, -90, 0, gh), 4, 4);
        p.fill(colorsLeft.z[0], colorsLeft.z[1], colorsLeft.z[2]);
        p.ellipse(mx, gy + p.map(dL.orientZ, 90, -90, 0, gh), 4, 4);
    }
}

function draw3D(p, section, hand) {
    let data = hand === 'right' ? section.dataRight : section.dataLeft;
    if (data.length === 0) return;
    
    let d = data[Math.min(section.currentIndex, data.length - 1)];
    let heading = p.radians(d.orientX);
    let roll = p.radians(d.orientY);
    let pitch = p.radians(d.orientZ);
    
    let boxColor = hand === 'right' ? [70, 120, 170] : [170, 100, 70];
    let strokeColor = hand === 'right' ? [100, 150, 200] : [200, 130, 100];
    
    p.ambientLight(100);
    p.directionalLight(255, 255, 255, 0.5, 0.5, -1);
    
    p.push();
    p.translate(0, 50, 0);
    p.rotateX(p.HALF_PI);
    p.stroke(50);
    p.strokeWeight(1);
    p.noFill();
    for (let i = -2; i <= 2; i++) {
        p.line(i * 15, -30, i * 15, 30);
        p.line(-30, i * 15, 30, i * 15);
    }
    p.pop();
    
    p.strokeWeight(2);
    p.stroke(255, 80, 80);
    p.line(0, 0, 0, 35, 0, 0);
    p.stroke(80, 255, 80);
    p.line(0, 0, 0, 0, -35, 0);
    p.stroke(80, 80, 255);
    p.line(0, 0, 0, 0, 0, 35);
    
    p.push();
    p.rotateY(-heading);
    p.rotateZ(pitch);
    p.rotateX(roll);
    
    p.fill(boxColor[0], boxColor[1], boxColor[2]);
    p.stroke(strokeColor[0], strokeColor[1], strokeColor[2]);
    p.strokeWeight(1);
    p.box(35, 10, 22);
    
    p.push();
    p.translate(0, -7, 0);
    p.fill(255, 80, 80);
    p.noStroke();
    p.box(8, 2, 8);
    p.pop();
    
    p.push();
    p.translate(22, 0, 0);
    p.rotateZ(-p.HALF_PI);
    p.fill(255, 200, 80);
    p.noStroke();
    p.cone(5, 10);
    p.pop();
    
    p.pop();
    
    // Hand label
    p.push();
    p.translate(-p.width/2, -p.height/2);
    p.fill(150);
    p.textSize(10);
    p.textAlign(p.CENTER);
    let labelX = hand === 'right' ? 60 : 220;
    p.text(hand === 'right' ? 'R' : 'L', p.width/2 + labelX, 125);
    p.pop();
}

function drawBrightnessGraph(p, section) {
    let gx = 380;
    let gy = 200;
    let gw = 280;
    let gh = 160;
    
    // Background
    p.fill(40);
    p.noStroke();
    p.rect(gx, gy, gw, gh);
    
    // Grid
    p.stroke(60);
    p.strokeWeight(1);
    for (let i = 0; i <= 4; i++) {
        let y = gy + (gh / 4) * i;
        p.line(gx, y, gx + gw, y);
    }
    
    // Right hand brightness line (green)
    if (section.brightnessHistoryRight.length > 1) {
        p.stroke(100, 255, 150);
        p.strokeWeight(2);
        p.noFill();
        p.beginShape();
        for (let i = 0; i < section.brightnessHistoryRight.length; i++) {
            let x = p.map(i, 0, section.brightnessHistoryRight.length - 1, gx, gx + gw);
            let y = p.map(section.brightnessHistoryRight[i], 0, 255, gy + gh, gy);
            p.vertex(x, y);
        }
        p.endShape();
    }
    
    // Left hand brightness line (orange)
    if (section.brightnessHistoryLeft.length > 1) {
        p.stroke(255, 180, 100);
        p.strokeWeight(2);
        p.noFill();
        p.beginShape();
        for (let i = 0; i < section.brightnessHistoryLeft.length; i++) {
            let x = p.map(i, 0, section.brightnessHistoryLeft.length - 1, gx, gx + gw);
            let y = p.map(section.brightnessHistoryLeft[i], 0, 255, gy + gh, gy);
            p.vertex(x, y);
        }
        p.endShape();
    }
    
    // Current brightness indicators
    if (section.brightnessHistoryRight.length > 0) {
        let lastR = section.brightnessHistoryRight[section.brightnessHistoryRight.length - 1];
        let yR = p.map(lastR, 0, 255, gy + gh, gy);
        p.fill(100, 255, 150);
        p.noStroke();
        p.ellipse(gx + gw, yR, 8, 8);
    }
    if (section.brightnessHistoryLeft.length > 0) {
        let lastL = section.brightnessHistoryLeft[section.brightnessHistoryLeft.length - 1];
        let yL = p.map(lastL, 0, 255, gy + gh, gy);
        p.fill(255, 180, 100);
        p.noStroke();
        p.ellipse(gx + gw, yL, 6, 6);
    }
    
    // Labels
    p.fill(100, 255, 150);
    p.textSize(10);
    p.textAlign(p.LEFT);
    p.text('R', gx + 5, gy + 12);
    p.fill(255, 180, 100);
    p.text('L', gx + 20, gy + 12);
}

function bindSectionEvents(section) {
    let el = section.element;
    
    // Page navigation
    el.querySelector('.nav-prev').addEventListener('click', () => {
        if (section.currentPage > 0) {
            section.currentPage--;
            updateSectionPage(section);
        }
    });
    
    el.querySelector('.nav-next').addEventListener('click', () => {
        if (section.currentPage < 2) {
            section.currentPage++;
            updateSectionPage(section);
        }
    });
    
    el.querySelector('.play-btn').addEventListener('click', () => {
        section.isPlaying = !section.isPlaying;
        section.magIsPlaying = section.isPlaying; // Sync with Magnitude view
        if (section.isPlaying) {
            // Calculate playStartTime based on current frame position
            let currentFrameMs = timestampToMs(section.dataRight[section.currentIndex].timestamp);
            let firstFrameMs = timestampToMs(section.dataRight[0].timestamp);
            let offsetMs = currentFrameMs - firstFrameMs;
            section.playStartTime = Date.now() - offsetMs;
        }
        el.querySelector('.play-btn').textContent = section.isPlaying ? 'Pause' : 'Play';
        // Sync Magnitude play button
        let magPlayBtn = el.querySelector('.mag-play-btn');
        if (magPlayBtn) magPlayBtn.textContent = section.isPlaying ? '⏸' : '▶';
    });
    
    el.querySelector('.reset-btn').addEventListener('click', () => {
        section.currentIndex = 0;
        section.isPlaying = false;
        section.magIsPlaying = false; // Sync with Magnitude view
        section.playStartTime = 0;
        el.querySelector('.play-btn').textContent = 'Play';
        // Sync Magnitude play button and timeline
        let magPlayBtn = el.querySelector('.mag-play-btn');
        if (magPlayBtn) magPlayBtn.textContent = '▶';
        let magTimeline = el.querySelector('.mag-timeline');
        if (magTimeline) magTimeline.value = 0;
        updateSectionDisplay(section);
    });
    
    el.querySelector('.collapse-btn').addEventListener('click', function() {
        let content = el.querySelector('.section-content');
        if (content.classList.contains('collapsed')) {
            content.classList.remove('collapsed');
            this.textContent = '-';
        } else {
            content.classList.add('collapsed');
            this.textContent = '+';
        }
    });
    
    el.querySelector('.delete-btn').addEventListener('click', () => {
        if (sections.length > 1) {
            section.p5.remove();
            sections = sections.filter(s => s.id !== section.id);
            el.remove();
        }
    });
    
    el.querySelector('.parse-btn').addEventListener('click', () => {
        parseData(section);
    });
    
    el.querySelector('.clear-btn').addEventListener('click', () => {
        clearData(section);
    });
}

function updateSectionDisplay(section) {
    let el = section.element;
    
    // Right hand data
    let dR = section.dataRight[Math.min(section.currentIndex, section.dataRight.length - 1)];
    el.querySelector('.valRX').textContent = dR.orientX.toFixed(2);
    el.querySelector('.valRY').textContent = dR.orientY.toFixed(2);
    el.querySelector('.valRZ').textContent = dR.orientZ.toFixed(2);
    
    // Left hand data
    if (section.dataLeft.length > 0) {
        let dL = section.dataLeft[Math.min(section.currentIndex, section.dataLeft.length - 1)];
        el.querySelector('.valLX').textContent = dL.orientX.toFixed(2);
        el.querySelector('.valLY').textContent = dL.orientY.toFixed(2);
        el.querySelector('.valLZ').textContent = dL.orientZ.toFixed(2);
    }
    
    el.querySelector('.valTime').textContent = dR.timestamp;
}

function parseData(section) {
    let el = section.element;
    let textareaRight = el.querySelector('.serial-data-right');
    let textareaLeft = el.querySelector('.serial-data-left');
    let status = el.querySelector('.parse-status');
    
    function parseCSVData(rawText) {
        let result = [];
        rawText = rawText.trim().replace(/\n/g, '');
        
        // Format: timestamp,orientX,orientY,orientZ,accelX,accelY,accelZ,speed,sys,gyro,accel,mag,
        let framePattern = /(\d{2}:\d{2}:\d{2}\.\d{3}),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),([-\d.]+),(\d+),(\d+),(\d+),(\d+),/g;
        let match;
        
        while ((match = framePattern.exec(rawText)) !== null) {
            result.push({
                timestamp: match[1],
                orientX: parseFloat(match[2]),
                orientY: parseFloat(match[3]),
                orientZ: parseFloat(match[4]),
                accelX: parseFloat(match[5]),
                accelY: parseFloat(match[6]),
                accelZ: parseFloat(match[7]),
                speed: parseFloat(match[8]),
                calSys: parseInt(match[9]),
                calGyro: parseInt(match[10]),
                calAccel: parseInt(match[11]),
                calMag: parseInt(match[12])
            });
        }
        return result;
    }
    
    let rightData = textareaRight && textareaRight.value.trim() ? parseCSVData(textareaRight.value) : [];
    let leftData = textareaLeft && textareaLeft.value.trim() ? parseCSVData(textareaLeft.value) : [];
    
    if (rightData.length === 0 && leftData.length === 0) {
        status.textContent = 'No valid data found';
        return;
    }
    
    // If only one hand has data, use it for both (or keep empty)
    section.dataRight = rightData.length > 0 ? rightData : JSON.parse(JSON.stringify(defaultData));
    section.dataLeft = leftData.length > 0 ? leftData : [];
    
    section.currentIndex = 0;
    section.isPlaying = false;
    el.querySelector('.play-btn').textContent = 'Play';
    updateSectionDisplay(section);
    status.textContent = `R:${rightData.length} L:${leftData.length} points`;
}

function clearData(section) {
    let el = section.element;
    let textareaRight = el.querySelector('.serial-data-right');
    let textareaLeft = el.querySelector('.serial-data-left');
    if (textareaRight) textareaRight.value = '';
    if (textareaLeft) textareaLeft.value = '';
    el.querySelector('.parse-status').textContent = '';
    
    section.dataRight = JSON.parse(JSON.stringify(defaultData));
    section.dataLeft = [];
    section.currentIndex = 0;
    section.isPlaying = false;
    el.querySelector('.play-btn').textContent = 'Play';
    updateSectionDisplay(section);
}

function setupAddButton() {
    document.querySelector('.add-section-btn').addEventListener('click', addNewSection);
}

function addNewSection() {
    let grid = document.getElementById('sections-grid');
    let addSlot = document.querySelector('.add-section-slot');
    
    let html = `
        <section class="viz-section" data-section-id="${sectionCounter}">
            <div class="section-header">
                <input type="text" class="section-title" value="Section ${sectionCounter + 1}" placeholder="Section Title">
                <div class="header-buttons">
                    <button class="delete-btn">x</button>
                    <button class="collapse-btn">-</button>
                </div>
            </div>
            <div class="section-content">
                <!-- Section Page Navigation -->
                <div class="section-page-nav">
                    <button class="section-nav-btn nav-prev" disabled>◀</button>
                    <span class="section-page-indicator">Main</span>
                    <button class="section-nav-btn nav-next">▶</button>
                </div>
                
                <!-- Page 0: Main View -->
                <div class="section-page main-view active">
                    <div class="controls">
                        <button class="play-btn">Play</button>
                        <button class="reset-btn">Reset</button>
                    </div>
                    <div class="info-display">
                        <div class="value-box vx">RX: <span class="valRX">--</span></div>
                        <div class="value-box vy">RY: <span class="valRY">--</span></div>
                        <div class="value-box vz">RZ: <span class="valRZ">--</span></div>
                        <div class="value-box vx2">LX: <span class="valLX">--</span></div>
                        <div class="value-box vy2">LY: <span class="valLY">--</span></div>
                        <div class="value-box vz2">LZ: <span class="valLZ">--</span></div>
                        <div class="value-box vt">T: <span class="valTime">--</span></div>
                    </div>
                    <div class="canvas-container"></div>
                    <div class="legend">
                        <span><span class="legend-color" style="background:#ff6464;"></span>RX</span>
                        <span><span class="legend-color" style="background:#64c8c8;"></span>RY</span>
                        <span><span class="legend-color" style="background:#ffe664;"></span>RZ</span>
                        <span><span class="legend-color" style="background:#ff9696;"></span>LX</span>
                        <span><span class="legend-color" style="background:#96dada;"></span>LY</span>
                        <span><span class="legend-color" style="background:#fff096;"></span>LZ</span>
                    </div>
                    <div class="data-input-dual">
                        <div class="data-input-half">
                            <h4>Right Hand</h4>
                            <textarea class="serial-data-right" placeholder="00:00:02.349,356.81,26.94,-77.00,1.02,-0.93,0.61,0.24,0,0,0,0,"></textarea>
                        </div>
                        <div class="data-input-half">
                            <h4>Left Hand</h4>
                            <textarea class="serial-data-left" placeholder="00:00:02.349,356.81,26.94,-77.00,1.02,-0.93,0.61,0.24,0,0,0,0,"></textarea>
                        </div>
                    </div>
                    <div class="input-buttons">
                        <button class="parse-btn">Load Data</button>
                        <button class="clear-btn">Clear</button>
                    </div>
                    <div class="parse-status"></div>
                </div>
                
                <!-- Page 1: Heatmap View -->
                <div class="section-page heatmap-view">
                    <div class="heatmap-grid">
                        <div class="heatmap-panel" data-axis="rx">
                            <h4>Right X</h4>
                            <div class="heatmap-canvas"></div>
                            <div class="heatmap-legend"><span class="hm-min">-</span><div class="hm-gradient"></div><span class="hm-max">-</span></div>
                        </div>
                        <div class="heatmap-panel" data-axis="ry">
                            <h4>Right Y</h4>
                            <div class="heatmap-canvas"></div>
                            <div class="heatmap-legend"><span class="hm-min">-</span><div class="hm-gradient"></div><span class="hm-max">-</span></div>
                        </div>
                        <div class="heatmap-panel" data-axis="rz">
                            <h4>Right Z</h4>
                            <div class="heatmap-canvas"></div>
                            <div class="heatmap-legend"><span class="hm-min">-</span><div class="hm-gradient"></div><span class="hm-max">-</span></div>
                        </div>
                        <div class="heatmap-panel" data-axis="rxyz">
                            <h4>Right XYZ</h4>
                            <div class="heatmap-canvas"></div>
                            <div class="heatmap-legend"><span class="hm-min">-</span><div class="hm-gradient"></div><span class="hm-max">-</span></div>
                        </div>
                        <div class="heatmap-panel left-panel" data-axis="lx">
                            <h4>Left X</h4>
                            <div class="heatmap-canvas"></div>
                            <div class="heatmap-legend"><span class="hm-min">-</span><div class="hm-gradient"></div><span class="hm-max">-</span></div>
                        </div>
                        <div class="heatmap-panel left-panel" data-axis="ly">
                            <h4>Left Y</h4>
                            <div class="heatmap-canvas"></div>
                            <div class="heatmap-legend"><span class="hm-min">-</span><div class="hm-gradient"></div><span class="hm-max">-</span></div>
                        </div>
                        <div class="heatmap-panel left-panel" data-axis="lz">
                            <h4>Left Z</h4>
                            <div class="heatmap-canvas"></div>
                            <div class="heatmap-legend"><span class="hm-min">-</span><div class="hm-gradient"></div><span class="hm-max">-</span></div>
                        </div>
                        <div class="heatmap-panel left-panel" data-axis="lxyz">
                            <h4>Left XYZ</h4>
                            <div class="heatmap-canvas"></div>
                            <div class="heatmap-legend"><span class="hm-min">-</span><div class="hm-gradient"></div><span class="hm-max">-</span></div>
                        </div>
                    </div>
                    <div class="heatmap-tooltip"></div>
                </div>
                
                <!-- Page 2: Magnitude View -->
                <div class="section-page magnitude-view">
                    <div class="magnitude-canvas-single"></div>
                    <div class="magnitude-controls">
                        <div class="mag-playback">
                            <button class="mag-play-btn">▶</button>
                            <input type="range" class="mag-timeline" min="0" max="100" value="0">
                            <span class="mag-time">0:00</span>
                        </div>
                        <div class="mag-speed">
                            <span class="mag-speed-label">Speed:</span>
                            <input type="range" class="mag-speed-slider" min="0" max="4" step="1" value="2">
                            <span class="mag-speed-val">1x</span>
                        </div>
                    </div>
                    <div class="magnitude-info">
                        <span class="mag-frame">Frame: <span class="mag-frame-num">0</span></span>
                        <span class="mag-right-val">R: <span class="mag-r">--</span></span>
                        <span class="mag-left-val">L: <span class="mag-l">--</span></span>
                    </div>
                </div>
            </div>
        </section>
    `;
    
    addSlot.insertAdjacentHTML('beforebegin', html);
    
    let newEl = grid.querySelector(`[data-section-id="${sectionCounter}"]`);
    createSectionInstance(newEl, sectionCounter);
    sectionCounter++;
}

// Section Page Navigation
function updateSectionPage(section) {
    let el = section.element;
    let mainView = el.querySelector('.main-view');
    let heatmapView = el.querySelector('.heatmap-view');
    let magnitudeView = el.querySelector('.magnitude-view');
    let navPrev = el.querySelector('.nav-prev');
    let navNext = el.querySelector('.nav-next');
    let indicator = el.querySelector('.section-page-indicator');
    
    // Hide all pages
    mainView.classList.remove('active');
    heatmapView.classList.remove('active');
    if (magnitudeView) magnitudeView.classList.remove('active');
    
    if (section.currentPage === 0) {
        mainView.classList.add('active');
        navPrev.disabled = true;
        navNext.disabled = false;
        indicator.textContent = 'Main';
    } else if (section.currentPage === 1) {
        heatmapView.classList.add('active');
        navPrev.disabled = false;
        navNext.disabled = false;
        indicator.textContent = 'Heatmap';
        initSectionHeatmaps(section);
    } else if (section.currentPage === 2) {
        if (magnitudeView) magnitudeView.classList.add('active');
        navPrev.disabled = false;
        navNext.disabled = true;
        indicator.textContent = 'Magnitude';
        initMagnitudeView(section);
    }
}

// Magnitude View Functions
function initMagnitudeView(section) {
    let el = section.element;
    
    // Clear existing magnitude instance
    if (section.magnitudeInstance && section.magnitudeInstance.remove) {
        section.magnitudeInstance.remove();
    }
    
    let container = el.querySelector('.magnitude-canvas-single');
    
    if (container) {
        container.innerHTML = '';
        section.magnitudeInstance = new p5(sketch => magnitudeSketchCombined(sketch, section), container);
    }
    
    // Bind magnitude controls
    bindMagnitudeControls(section);
}

function bindMagnitudeControls(section) {
    let el = section.element;
    let playBtn = el.querySelector('.mag-play-btn');
    let timeline = el.querySelector('.mag-timeline');
    let timeDisplay = el.querySelector('.mag-time');
    let speedSlider = el.querySelector('.mag-speed-slider');
    let speedVal = el.querySelector('.mag-speed-val');
    
    if (!playBtn || !timeline || !speedSlider) return;
    
    // Speed values: 0=0.25x, 1=0.5x, 2=1x, 3=2x, 4=4x
    const speedValues = [0.25, 0.5, 1, 2, 4];
    
    // Update timeline max based on data length
    let maxFrames = Math.max(section.dataRight.length, section.dataLeft.length) - 1;
    timeline.max = maxFrames;
    timeline.value = section.currentIndex;
    
    // Play/Pause button
    playBtn.onclick = function() {
        section.magIsPlaying = !section.magIsPlaying;
        section.isPlaying = section.magIsPlaying; // Sync with Main view
        playBtn.textContent = section.magIsPlaying ? '⏸' : '▶';
        
        if (section.magIsPlaying) {
            // Calculate start time accounting for current position
            let currentFrameMs = timestampToMs(section.dataRight[section.currentIndex].timestamp);
            let firstFrameMs = timestampToMs(section.dataRight[0].timestamp);
            let offsetMs = (currentFrameMs - firstFrameMs) / section.magPlaySpeed;
            section.playStartTime = Date.now() - offsetMs;
        }
        // Sync Main view play button
        let mainPlayBtn = section.element.querySelector('.play-btn');
        if (mainPlayBtn) mainPlayBtn.textContent = section.isPlaying ? 'Pause' : 'Play';
    };
    
    // Timeline slider
    timeline.oninput = function() {
        section.currentIndex = parseInt(this.value);
        updateMagnitudeTimeDisplay(section);
        
        // Reset play start time if playing
        if (section.magIsPlaying) {
            let currentFrameMs = timestampToMs(section.dataRight[section.currentIndex].timestamp);
            let firstFrameMs = timestampToMs(section.dataRight[0].timestamp);
            let offsetMs = (currentFrameMs - firstFrameMs) / section.magPlaySpeed;
            section.playStartTime = Date.now() - offsetMs;
        }
    };
    
    // Speed slider
    speedSlider.oninput = function() {
        let speedIndex = parseInt(this.value);
        section.magPlaySpeed = speedValues[speedIndex];
        speedVal.textContent = section.magPlaySpeed + 'x';
        
        // Recalculate play start time to maintain position
        if (section.magIsPlaying) {
            let currentFrameMs = timestampToMs(section.dataRight[section.currentIndex].timestamp);
            let firstFrameMs = timestampToMs(section.dataRight[0].timestamp);
            let offsetMs = (currentFrameMs - firstFrameMs) / section.magPlaySpeed;
            section.playStartTime = Date.now() - offsetMs;
        }
    };
    
    // Initialize display
    updateMagnitudeTimeDisplay(section);
}

function updateMagnitudeTimeDisplay(section) {
    let el = section.element;
    let timeDisplay = el.querySelector('.mag-time');
    let timeline = el.querySelector('.mag-timeline');
    
    if (!timeDisplay || !timeline) return;
    
    let d = section.dataRight[Math.min(section.currentIndex, section.dataRight.length - 1)];
    if (d && d.timestamp) {
        // Extract minutes:seconds from timestamp
        let parts = d.timestamp.split(':');
        timeDisplay.textContent = parts[1] + ':' + parts[2].split('.')[0];
    }
    timeline.value = section.currentIndex;
}

function magnitudeSketchCombined(p, section) {
    let canvasWidth, canvasHeight;
    let parentEl;
    
    // Separate state for each hand
    let currentCirclesRight = 0;
    let currentCirclesLeft = 0;
    let maxCircles = 100;
    let circleSpacing = 5;
    
    // Trail history for afterimage effect (separate for each hand)
    let trailHistoryRight = [];
    let trailHistoryLeft = [];
    let maxTrailLength = 8;
    
    p.setup = function() {
        parentEl = p.canvas ? p.canvas.parentElement : null;
        if (!parentEl) {
            canvasWidth = 700;
            canvasHeight = 400;
        } else {
            canvasWidth = parentEl.offsetWidth || 700;
            canvasHeight = parentEl.offsetHeight || 400;
        }
        let canvas = p.createCanvas(canvasWidth, canvasHeight);
        canvas.style('display', 'block');
        p.noFill();
        p.strokeWeight(0.5);
    };
    
    p.windowResized = function() {
        if (parentEl) {
            canvasWidth = parentEl.offsetWidth || 700;
            canvasHeight = parentEl.offsetHeight || 400;
            p.resizeCanvas(canvasWidth, canvasHeight);
        }
    };
    
    p.draw = function() {
        p.background(0);
        
        // Magnitude view playback (with speed control)
        if (section.magIsPlaying && section.dataRight.length > 0) {
            let currentTime = Date.now();
            let elapsedTime = (currentTime - section.playStartTime) * section.magPlaySpeed;
            let dataStartMs = timestampToMs(section.dataRight[0].timestamp);
            let targetTimeMs = dataStartMs + elapsedTime;
            
            // Find the frame that matches current playback time
            let newIndex = section.currentIndex;
            let maxLen = Math.max(section.dataRight.length, section.dataLeft.length);
            for (let i = 0; i < maxLen; i++) {
                let frameTimeMs = timestampToMs(section.dataRight[Math.min(i, section.dataRight.length - 1)].timestamp);
                if (frameTimeMs <= targetTimeMs) {
                    newIndex = i;
                } else {
                    break;
                }
            }
            
            // Process each skipped frame individually to maintain accurate brightness
            if (newIndex !== section.currentIndex) {
                let startIdx = section.currentIndex;
                let endIdx = newIndex;
                
                // Process each frame one by one
                for (let i = startIdx; i < endIdx; i++) {
                    // Calculate brightness change for this single frame step
                    updateMovementBrightnessForFrame(section, i, i + 1);
                    
                    // Apply decay for each frame (scaled by speed so decay feels the same)
                    section.targetBrightnessRight *= 0.80;
                    section.targetBrightnessLeft *= 0.80;
                }
                
                section.currentIndex = newIndex;
                updateMagnitudeTimeDisplay(section);
            }
            
            // Loop back to start when finished
            if (section.currentIndex >= maxLen - 1) {
                section.currentIndex = 0;
                section.playStartTime = Date.now();
                // Reset brightness on loop
                section.targetBrightnessRight = 0;
                section.targetBrightnessLeft = 0;
                section.currentBrightnessRight = 0;
                section.currentBrightnessLeft = 0;
            }
        }
        
        // Apply smooth lerp for visual brightness (independent of playback)
        section.currentBrightnessRight = p.lerp(section.currentBrightnessRight, section.targetBrightnessRight, 0.15);
        section.currentBrightnessLeft = p.lerp(section.currentBrightnessLeft, section.targetBrightnessLeft, 0.15);
        
        let centerY = p.height / 2;
        // Position circles so they overlap in the center
        let centerXRight = p.width * 0.35;
        let centerXLeft = p.width * 0.65;
        
        // === RIGHT HAND ===
        let brightnessRight = section.currentBrightnessRight || 0;
        let targetCirclesRight = p.map(brightnessRight, 0, 255, 1, maxCircles);
        targetCirclesRight = p.constrain(targetCirclesRight, 1, maxCircles);
        currentCirclesRight = p.lerp(currentCirclesRight, targetCirclesRight, 0.2);
        let numCirclesRight = Math.round(currentCirclesRight);
        
        // Add to trail history
        if (p.frameCount % 2 === 0) {
            trailHistoryRight.unshift(numCirclesRight);
            if (trailHistoryRight.length > maxTrailLength) trailHistoryRight.pop();
        }
        
        // === LEFT HAND ===
        let brightnessLeft = section.currentBrightnessLeft || 0;
        let targetCirclesLeft = p.map(brightnessLeft, 0, 255, 1, maxCircles);
        targetCirclesLeft = p.constrain(targetCirclesLeft, 1, maxCircles);
        currentCirclesLeft = p.lerp(currentCirclesLeft, targetCirclesLeft, 0.2);
        let numCirclesLeft = Math.round(currentCirclesLeft);
        
        // Add to trail history
        if (p.frameCount % 2 === 0) {
            trailHistoryLeft.unshift(numCirclesLeft);
            if (trailHistoryLeft.length > maxTrailLength) trailHistoryLeft.pop();
        }
        
        // Draw afterimages for RIGHT hand (red)
        p.noFill();
        for (let t = trailHistoryRight.length - 1; t >= 1; t--) {
            let trailCircles = trailHistoryRight[t];
            let alpha = p.map(t, trailHistoryRight.length - 1, 0, 15, 150);
            p.stroke(255, 80, 80, alpha); // Red
            for (let i = 1; i <= trailCircles; i++) {
                p.ellipse(centerXRight, centerY, i * circleSpacing * 2, i * circleSpacing * 2);
            }
        }
        
        // Draw afterimages for LEFT hand (blue)
        for (let t = trailHistoryLeft.length - 1; t >= 1; t--) {
            let trailCircles = trailHistoryLeft[t];
            let alpha = p.map(t, trailHistoryLeft.length - 1, 0, 15, 150);
            p.stroke(80, 150, 255, alpha); // Blue
            for (let i = 1; i <= trailCircles; i++) {
                p.ellipse(centerXLeft, centerY, i * circleSpacing * 2, i * circleSpacing * 2);
            }
        }
        
        // Draw current circles for RIGHT hand (red, outer circles fade, inner circles solid)
        p.noFill();
        for (let i = 1; i <= numCirclesRight; i++) {
            // Outer 3 circles are semi-transparent, fading outward
            let fromEnd = numCirclesRight - i; // 0 for outermost, increases inward
            if (fromEnd < 10) {
                let alpha = p.map(fromEnd, 0, 9, 20, 240); // outermost=80, 3rd from end=220
                p.stroke(255, 80, 80, alpha); // Red
            } else {
                p.stroke(255, 80, 80); // Red
            }
            p.ellipse(centerXRight, centerY, i * circleSpacing * 2, i * circleSpacing * 2);
        }
        
        // Draw current circles for LEFT hand (blue, outer circles fade, inner circles solid)
        for (let i = 1; i <= numCirclesLeft; i++) {
            let fromEnd = numCirclesLeft - i;
            if (fromEnd < 10) {
                let alpha = p.map(fromEnd, 0, 9, 20, 240);
                p.stroke(80, 150, 255, alpha); // Blue
            } else {
                p.stroke(80, 150, 255); // Blue
            }
            p.ellipse(centerXLeft, centerY, i * circleSpacing * 2, i * circleSpacing * 2);
        }
        
        // Draw center dots
        p.fill(255, 80, 80); // Red for right
        p.noStroke();
        p.ellipse(centerXRight, centerY, 1, 1);
        p.fill(80, 150, 255); // Blue for left
        p.ellipse(centerXLeft, centerY, 1, 1);
        
        // Update info display
        let el = section.element;
        el.querySelector('.mag-frame-num').textContent = section.currentIndex;
        el.querySelector('.mag-r').textContent = brightnessRight.toFixed(0);
        el.querySelector('.mag-l').textContent = brightnessLeft.toFixed(0);
    };
}

// Heatmap Functions for Section
function initSectionHeatmaps(section) {
    let el = section.element;
    
    // Clear existing heatmap instances
    Object.values(section.heatmapInstances).forEach(instance => {
        if (instance && instance.remove) instance.remove();
    });
    section.heatmapInstances = {};
    
    let tooltipEl = el.querySelector('.heatmap-tooltip');
    
    // Right hand heatmaps
    ['rx', 'ry', 'rz', 'rxyz'].forEach(axis => {
        let panel = el.querySelector(`.heatmap-panel[data-axis="${axis}"]`);
        if (!panel) return;
        let container = panel.querySelector('.heatmap-canvas');
        container.innerHTML = '';
        
        let data = section.dataRight;
        let dataKey = axis === 'rxyz' ? 'xyz' : 'orient' + axis.charAt(1).toUpperCase();
        let p5Instance = new p5(sketch => heatmapSketch(sketch, data, dataKey, tooltipEl, 'R'), container);
        section.heatmapInstances[axis] = p5Instance;
        
        // Update legend
        let values;
        if (dataKey === 'xyz') {
            values = data.map(d => Math.sqrt(d.orientX * d.orientX + d.orientY * d.orientY + d.orientZ * d.orientZ));
        } else {
            values = data.map(d => d[dataKey]);
        }
        let minVal = Math.min(...values).toFixed(1);
        let maxVal = Math.max(...values).toFixed(1);
        panel.querySelector('.hm-min').textContent = minVal;
        panel.querySelector('.hm-max').textContent = maxVal;
    });
    
    // Left hand heatmaps (only if data exists)
    if (section.dataLeft.length > 0) {
        ['lx', 'ly', 'lz', 'lxyz'].forEach(axis => {
            let panel = el.querySelector(`.heatmap-panel[data-axis="${axis}"]`);
            if (!panel) return;
            let container = panel.querySelector('.heatmap-canvas');
            container.innerHTML = '';
            
            let data = section.dataLeft;
            let dataKey = axis === 'lxyz' ? 'xyz' : 'orient' + axis.charAt(1).toUpperCase();
            let p5Instance = new p5(sketch => heatmapSketch(sketch, data, dataKey, tooltipEl, 'L'), container);
            section.heatmapInstances[axis] = p5Instance;
            
            // Update legend
            let values;
            if (dataKey === 'xyz') {
                values = data.map(d => Math.sqrt(d.orientX * d.orientX + d.orientY * d.orientY + d.orientZ * d.orientZ));
            } else {
                values = data.map(d => d[dataKey]);
            }
            let minVal = Math.min(...values).toFixed(1);
            let maxVal = Math.max(...values).toFixed(1);
            panel.querySelector('.hm-min').textContent = minVal;
            panel.querySelector('.hm-max').textContent = maxVal;
        });
    }
}

function heatmapSketch(p, data, axis, tooltipEl, handLabel) {
    if (data.length === 0) return;
    
    let cellSize = 10;
    let cols = Math.ceil(Math.sqrt(data.length * 1.5));
    let rows = Math.ceil(data.length / cols);
    let offsetX = 20;
    let offsetY = 15;
    let canvasWidth = cols * cellSize + offsetX + 10;
    let canvasHeight = rows * cellSize + offsetY + 10;
    
    // Calculate min/max for this axis
    let values = [];
    if (axis === 'xyz') {
        values = data.map(d => Math.sqrt(d.orientX * d.orientX + d.orientY * d.orientY + d.orientZ * d.orientZ));
    } else {
        values = data.map(d => d[axis]);
    }
    let minVal = Math.min(...values);
    let maxVal = Math.max(...values);
    
    p.setup = function() {
        p.createCanvas(canvasWidth, canvasHeight);
        p.noLoop();
    };
    
    p.draw = function() {
        p.background(0);
        
        // Draw heatmap cells
        for (let i = 0; i < data.length; i++) {
            let col = i % cols;
            let row = Math.floor(i / cols);
            let x = offsetX + col * cellSize;
            let y = offsetY + row * cellSize;
            
            let value;
            if (axis === 'xyz') {
                value = Math.sqrt(data[i].orientX * data[i].orientX + data[i].orientY * data[i].orientY + data[i].orientZ * data[i].orientZ);
            } else {
                value = data[i][axis];
            }
            
            let color = valueToColor(p, value, minVal, maxVal);
            
            p.fill(color);
            p.stroke(20);
            p.strokeWeight(1);
            p.rect(x, y, cellSize - 1, cellSize - 1);
        }
        
        // Row labels
        p.fill(80);
        p.textSize(7);
        p.textAlign(p.RIGHT);
        for (let row = 0; row < rows; row += 2) {
            let frameStart = row * cols;
            if (frameStart < data.length) {
                p.text(frameStart, offsetX - 2, offsetY + row * cellSize + cellSize - 2);
            }
        }
    };
    
    p.mouseMoved = function() {
        let col = Math.floor((p.mouseX - offsetX) / cellSize);
        let row = Math.floor((p.mouseY - offsetY) / cellSize);
        let index = row * cols + col;
        
        if (index >= 0 && index < data.length && col >= 0 && col < cols && row >= 0 && row < rows) {
            let d = data[index];
            let value;
            let displayAxis = axis === 'xyz' ? 'XYZ' : axis.replace('orient', '');
            if (axis === 'xyz') {
                value = Math.sqrt(d.orientX * d.orientX + d.orientY * d.orientY + d.orientZ * d.orientZ).toFixed(2);
            } else {
                value = d[axis].toFixed(2);
            }
            tooltipEl.textContent = `${handLabel} | Frame ${index} | ${d.timestamp} | ${displayAxis}: ${value}`;
        }
    };
}

function valueToColor(p, value, minVal, maxVal) {
    let range = maxVal - minVal;
    if (range === 0) range = 1;
    let normalized = (value - minVal) / range;
    
    let r, g, b;
    
    if (normalized < 0.5) {
        let t = normalized * 2;
        r = p.lerp(30, 255, t);
        g = p.lerp(58, 255, t);
        b = p.lerp(138, 255, t);
    } else {
        let t = (normalized - 0.5) * 2;
        r = p.lerp(255, 127, t);
        g = p.lerp(255, 29, t);
        b = p.lerp(255, 29, t);
    }
    
    return p.color(r, g, b);
}
