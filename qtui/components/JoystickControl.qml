import QtQuick 2.15

Item {
    id: root
    
    // Properties
    property real xPosition: 0    // -1.0 to 1.0
    property real yPosition: 0    // -1.0 to 1.0
    property real magnitude: 0    // 0.0 to 1.0
    property bool pressed: false
    
    // Size properties
    property real handleSize: width * 0.3
    
    // Signals
    signal positionChanged(real x, real y, real magnitude)
    signal released()
    
    width: 200
    height: 200
    
    // Background circle
    Rectangle {
        anchors.fill: parent
        radius: width / 2
        color: "transparent"
        border.width: 2
        border.color: Material.accent
        
        // Center crosshair
        Rectangle {
            width: 1
            height: parent.height
            anchors.centerIn: parent
            color: Material.accent
            opacity: 0.3
        }
        Rectangle {
            width: parent.width
            height: 1
            anchors.centerIn: parent
            color: Material.accent
            opacity: 0.3
        }
        
        // Joystick handle
        Rectangle {
            id: handle
            width: handleSize
            height: handleSize
            radius: width / 2
            color: pressed ? Material.accent : Material.foreground
            opacity: pressed ? 1.0 : 0.7
            
            // Center the handle when not being manipulated
            x: root.width / 2 - width / 2 + (pressed ? 0 : xPosition * (root.width - width) / 2)
            y: root.height / 2 - height / 2 + (pressed ? 0 : -yPosition * (root.height - height) / 2)
            
            Behavior on x {
                enabled: !pressed
                SpringAnimation { 
                    spring: 2.0
                    damping: 0.7
                }
            }
            Behavior on y {
                enabled: !pressed
                SpringAnimation { 
                    spring: 2.0
                    damping: 0.7
                }
            }
        }
    }
    
    // Touch area for controlling the joystick
    MouseArea {
        anchors.fill: parent
        
        onPressed: {
            root.pressed = true;
            updateJoystickPosition(mouse.x, mouse.y);
        }
        
        onReleased: {
            root.pressed = false;
            xPosition = 0;
            yPosition = 0;
            magnitude = 0;
            root.released();
        }
        
        onPositionChanged: {
            if (pressed) {
                updateJoystickPosition(mouse.x, mouse.y);
            }
        }
    }
    
    // Function to update joystick position
    function updateJoystickPosition(mouseX, mouseY) {
        // Calculate position relative to center
        var centerX = width / 2;
        var centerY = height / 2;
        
        var deltaX = mouseX - centerX;
        var deltaY = mouseY - centerY;
        
        // Calculate magnitude (distance from center)
        var mag = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        var maxRadius = Math.min(width, height) / 2;
        
        // Limit to boundaries
        if (mag > maxRadius) {
            deltaX = deltaX * maxRadius / mag;
            deltaY = deltaY * maxRadius / mag;
            mag = maxRadius;
        }
        
        // Position joystick handle
        handle.x = centerX + deltaX - handle.width / 2;
        handle.y = centerY + deltaY - handle.height / 2;
        
        // Update properties (-1 to 1 range)
        xPosition = deltaX / maxRadius;
        yPosition = -deltaY / maxRadius; // Invert Y axis
        magnitude = mag / maxRadius;
        
        // Emit signal
        positionChanged(xPosition, yPosition, magnitude);
    }
}
