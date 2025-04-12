import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Page {
    title: "Dashboard"
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 15
        spacing: 15
        
        // Visualization area
        VisualizationView {
            id: visualizationView
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 300
            
            // Update visualization based on IMU data
            pitch: hexapodContext.pitch
            roll: hexapodContext.roll
            imuData: hexapodContext.imuData
        }
        
        // Controls section
        RowLayout {
            Layout.fillWidth: true
            spacing: 20
            
            GroupBox {
                title: "Movement Controls"
                Layout.fillWidth: true
                enabled: hexapodContext.connected
                
                ColumnLayout {
                    anchors.fill: parent
                    spacing: 10
                    
                    JoystickControl {
                        id: joystick
                        Layout.alignment: Qt.AlignHCenter
                        width: 150
                        height: 150
                        
                        onPositionChanged: {
                            if (magnitude > 0.1) {
                                // Convert to direction and speed
                                let speed = Math.min(magnitude, 1.0);
                                
                                // Forward/backward has priority when joystick is mostly vertical
                                if (Math.abs(y) > Math.abs(x)) {
                                    if (y < 0) {
                                        hexapodContext.moveForward(speed);
                                    } else {
                                        hexapodContext.moveBackward(speed);
                                    }
                                } else {
                                    // Otherwise turn left/right
                                    if (x < 0) {
                                        hexapodContext.turnLeft(speed);
                                    } else {
                                        hexapodContext.turnRight(speed);
                                    }
                                }
                            } else {
                                hexapodContext.stop();
                            }
                        }
                        
                        onReleased: {
                            hexapodContext.stop();
                        }
                    }
                    
                    Label {
                        text: "Speed: " + (hexapodContext.speed * 100).toFixed(0) + "%"
                        Layout.alignment: Qt.AlignHCenter
                    }
                    
                    Slider {
                        id: speedSlider
                        from: 0.0
                        to: 1.0
                        value: hexapodContext.speed
                        stepSize: 0.01
                        Layout.fillWidth: true
                        
                        onValueChanged: {
                            hexapodContext.setSpeed(value);
                        }
                    }
                    
                    Button {
                        text: "Center All Legs"
                        Layout.fillWidth: true
                        onClicked: hexapodContext.centerAllLegs()
                    }
                }
            }
            
            GroupBox {
                title: "Orientation"
                Layout.fillWidth: true
                enabled: hexapodContext.connected
                
                GridLayout {
                    anchors.fill: parent
                    columns: 2
                    rowSpacing: 10
                    columnSpacing: 5
                    
                    Label { text: "Pitch:" }
                    Slider {
                        id: pitchSlider
                        from: -30
                        to: 30
                        value: hexapodContext.pitch
                        Layout.fillWidth: true
                        
                        onValueChanged: {
                            hexapodContext.setPitch(value);
                        }
                    }
                    
                    Label { text: "Roll:" }
                    Slider {
                        id: rollSlider
                        from: -30
                        to: 30
                        value: hexapodContext.roll
                        Layout.fillWidth: true
                        
                        onValueChanged: {
                            hexapodContext.setRoll(value);
                        }
                    }
                    
                    Label { text: "Balance:" }
                    Switch {
                        id: balanceSwitch
                        checked: hexapodContext.balanceEnabled
                        
                        onCheckedChanged: {
                            hexapodContext.setBalanceEnabled(checked);
                        }
                    }
                }
            }
        }
        
        // IMU data section
        GroupBox {
            title: "IMU Data"
            Layout.fillWidth: true
            
            GridLayout {
                anchors.fill: parent
                columns: 3
                rowSpacing: 5
                columnSpacing: 10
                
                Label { text: "Accel X:" }
                ProgressBar {
                    value: (hexapodContext.imuData.accelX + 1) / 2
                    Layout.fillWidth: true
                }
                Label { text: hexapodContext.imuData.accelX.toFixed(2) + " g" }
                
                Label { text: "Accel Y:" }
                ProgressBar {
                    value: (hexapodContext.imuData.accelY + 1) / 2
                    Layout.fillWidth: true
                }
                Label { text: hexapodContext.imuData.accelY.toFixed(2) + " g" }
                
                Label { text: "Accel Z:" }
                ProgressBar {
                    value: (hexapodContext.imuData.accelZ + 1) / 2
                    Layout.fillWidth: true
                }
                Label { text: hexapodContext.imuData.accelZ.toFixed(2) + " g" }
                
                Label { text: "Gyro X:" }
                ProgressBar {
                    value: (hexapodContext.imuData.gyroX + 180) / 360
                    Layout.fillWidth: true
                }
                Label { text: hexapodContext.imuData.gyroX.toFixed(1) + " °/s" }
                
                Label { text: "Gyro Y:" }
                ProgressBar {
                    value: (hexapodContext.imuData.gyroY + 180) / 360
                    Layout.fillWidth: true
                }
                Label { text: hexapodContext.imuData.gyroY.toFixed(1) + " °/s" }
                
                Label { text: "Gyro Z:" }
                ProgressBar {
                    value: (hexapodContext.imuData.gyroZ + 180) / 360
                    Layout.fillWidth: true
                }
                Label { text: hexapodContext.imuData.gyroZ.toFixed(1) + " °/s" }
            }
        }
    }
}
