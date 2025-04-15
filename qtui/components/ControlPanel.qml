import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Page {
    title: "Hexapod Control Panel"
    
    property double moveSpeed: 0.5

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 15
        spacing: 15
        
        // Movement controls section
        GroupBox {
            title: "Movement Controls"
            Layout.fillWidth: true
            enabled: hexapodContext.connected
            
            GridLayout {
                anchors.fill: parent
                columns: 3
                rows: 3
                rowSpacing: 10
                columnSpacing: 10
                
                // Empty cell for top-left
                Item { Layout.fillWidth: true }
                
                // Forward button
                Button {
                    text: "Forward"
                    icon.source: "qrc:/icons/arrow-up.png" 
                    Layout.fillWidth: true
                    
                    onPressed: {
                        hexapodContext.moveForward(moveSpeed)
                    }
                    onReleased: {
                        hexapodContext.stop()
                    }
                }
                
                // Empty cell for top-right
                Item { Layout.fillWidth: true }
                
                // Left button
                Button {
                    text: "Left"
                    icon.source: "qrc:/icons/arrow-left.png"
                    Layout.fillWidth: true
                    
                    onPressed: {
                        hexapodContext.turnLeft(moveSpeed)
                    }
                    onReleased: {
                        hexapodContext.stop()
                    }
                }
                
                // Stop button
                Button {
                    text: "Stop"
                    icon.source: "qrc:/icons/stop.png"
                    Layout.fillWidth: true
                    
                    onClicked: {
                        hexapodContext.stop()
                    }
                    Material.accent: Material.Red
                }
                
                // Right button
                Button {
                    text: "Right"
                    icon.source: "qrc:/icons/arrow-right.png"
                    Layout.fillWidth: true
                    
                    onPressed: {
                        hexapodContext.turnRight(moveSpeed)
                    }
                    onReleased: {
                        hexapodContext.stop()
                    }
                }
                
                // Empty cell for bottom-left
                Item { Layout.fillWidth: true }
                
                // Backward button
                Button {
                    text: "Backward"
                    icon.source: "qrc:/icons/arrow-down.png"
                    Layout.fillWidth: true
                    
                    onPressed: {
                        hexapodContext.moveBackward(moveSpeed)
                    }
                    onReleased: {
                        hexapodContext.stop()
                    }
                }
                
                // Empty cell for bottom-right
                Item { Layout.fillWidth: true }
            }
        }
        
        // Speed control section
        GroupBox {
            title: "Speed Control"
            Layout.fillWidth: true
            enabled: hexapodContext.connected
            
            ColumnLayout {
                anchors.fill: parent
                spacing: 10
                
                Label {
                    text: "Speed: " + Math.round(speedSlider.value * 100) + "%"
                    Layout.alignment: Qt.AlignHCenter
                }
                
                Slider {
                    id: speedSlider
                    from: 0.1
                    to: 1.0
                    value: hexapodContext.speed
                    stepSize: 0.05
                    Layout.fillWidth: true
                    
                    onValueChanged: {
                        moveSpeed = value
                        hexapodContext.setSpeed(value)
                    }
                }
                
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10
                    
                    Button {
                        text: "Slow"
                        Layout.fillWidth: true
                        onClicked: speedSlider.value = 0.2
                    }
                    
                    Button {
                        text: "Medium"
                        Layout.fillWidth: true
                        onClicked: speedSlider.value = 0.5
                    }
                    
                    Button {
                        text: "Fast"
                        Layout.fillWidth: true
                        onClicked: speedSlider.value = 0.8
                    }
                }
            }
        }
        
        // Positioning and actions section
        GroupBox {
            title: "Positioning & Actions"
            Layout.fillWidth: true
            enabled: hexapodContext.connected
            
            GridLayout {
                anchors.fill: parent
                columns: 2
                rowSpacing: 10
                columnSpacing: 10
                
                Button {
                    text: "Center All Legs"
                    icon.source: "qrc:/icons/center.png"
                    Layout.fillWidth: true
                    onClicked: hexapodContext.centerAllLegs()
                }
                
                Button {
                    text: "Balance Mode"
                    icon.source: "qrc:/icons/balance.png"
                    Layout.fillWidth: true
                    checkable: true
                    checked: hexapodContext.balanceEnabled
                    onToggled: hexapodContext.setBalanceEnabled(checked)
                }
            }
        }
        
        // Keyboard controls help
        GroupBox {
            title: "Keyboard Controls"
            Layout.fillWidth: true
            
            GridLayout {
                anchors.fill: parent
                columns: 2
                rowSpacing: 5
                columnSpacing: 10
                
                Label { text: "W:" }
                Label { text: "Move Forward" }
                
                Label { text: "S:" }
                Label { text: "Move Backward" }
                
                Label { text: "A:" }
                Label { text: "Turn Left" }
                
                Label { text: "D:" }
                Label { text: "Turn Right" }
                
                Label { text: "Space:" }
                Label { text: "Stop Movement" }
            }
        }
        
        // Spacer
        Item {
            Layout.fillHeight: true
        }
    }
}
