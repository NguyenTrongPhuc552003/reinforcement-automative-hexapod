import QtQuick 2.15
import QtQuick.Controls 2.15

// Placeholder for a more complex visualization
// In a real implementation, you might want to use Qt3D or a Canvas
Rectangle {
    id: root
    
    property double pitch: 0
    property double roll: 0
    property var imuData: ({})
    
    color: "#2A2A2A"
    border.color: "#4A4A4A"
    border.width: 1
    
    // Simple hexapod body visualization
    Rectangle {
        id: body
        width: Math.min(parent.width, parent.height) * 0.6
        height: width * 0.6
        radius: 10
        color: "#555555"
        border.width: 2
        border.color: "#777777"
        anchors.centerIn: parent
        
        // Apply rotation based on pitch and roll
        transform: [
            Rotation {
                origin.x: body.width / 2
                origin.y: body.height / 2
                angle: roll
                axis { x: 1; y: 0; z: 0 }
            },
            Rotation {
                origin.x: body.width / 2
                origin.y: body.height / 2
                angle: pitch
                axis { x: 0; y: 1; z: 0 }
            }
        ]
        
        // Direction indicator
        Rectangle {
            width: parent.width * 0.2
            height: width
            radius: width / 2
            color: "#FF5722"
            anchors {
                top: parent.top
                horizontalCenter: parent.horizontalCenter
                topMargin: 5
            }
        }
        
        // Legs placeholder
        Repeater {
            model: 6
            
            Rectangle {
                property int legIndex: index
                property real angle: index * 60
                
                color: "#333333"
                width: 10
                height: 40
                radius: 5
                
                x: body.width/2 + (body.width/2 - 5) * Math.cos(angle * Math.PI/180)
                y: body.height/2 + (body.height/2 - 5) * Math.sin(angle * Math.PI/180)
                
                transform: Rotation {
                    origin.x: 5
                    origin.y: 5
                    angle: index * 60
                }
            }
        }
    }
    
    // Small axes indicator in corner
    Item {
        id: axesIndicator
        width: 80
        height: 80
        anchors {
            right: parent.right
            bottom: parent.bottom
            margins: 10
        }
        
        // X axis (roll)
        Rectangle {
            id: xAxis
            width: 30
            height: 3
            color: "#FF5555"
            anchors.centerIn: parent
            
            transform: Rotation {
                origin.x: 0
                origin.y: 1.5
                angle: roll
                axis { x: 1; y: 0; z: 0 }
            }
        }
        
        // Y axis (pitch)
        Rectangle {
            id: yAxis
            width: 3
            height: 30
            color: "#55FF55"
            anchors.centerIn: parent
            
            transform: Rotation {
                origin.x: 1.5
                origin.y: 0
                angle: -pitch
                axis { x: 0; y: 1; z: 0 }
            }
        }
        
        // Z axis
        Rectangle {
            id: zAxis
            width: 3
            height: 30
            color: "#5555FF"
            anchors.centerIn: parent
            
            transform: Rotation {
                origin.x: 1.5
                origin.y: 15
                angle: 90
                axis { x: 1; y: 0; z: 0 }
            }
        }
    }
}
