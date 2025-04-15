import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import Qt.labs.settings 1.0

Page {
    title: "Connection"
    
    Settings {
        id: connectionSettings
        property string hostname: "localhost"
        property int port: 8080
    }
    
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 20
        
        GroupBox {
            title: "Server Connection"
            Layout.fillWidth: true
            
            GridLayout {
                anchors.fill: parent
                columns: 2
                rowSpacing: 15
                columnSpacing: 10
                
                Label { 
                    text: "Hostname:"
                    Layout.alignment: Qt.AlignRight
                }
                
                TextField {
                    id: hostnameField
                    text: connectionSettings.hostname
                    placeholderText: "Enter hostname or IP"
                    Layout.fillWidth: true
                    enabled: !hexapodContext.connected
                }
                
                Label { 
                    text: "Port:"
                    Layout.alignment: Qt.AlignRight
                }
                
                SpinBox {
                    id: portSpinBox
                    from: 1
                    to: 65535
                    value: connectionSettings.port
                    editable: true
                    enabled: !hexapodContext.connected
                    Layout.fillWidth: true
                }
                
                Item { width: 1; height: 1 } // Spacer
                
                RowLayout {
                    spacing: 10
                    Layout.fillWidth: true
                    
                    Button {
                        text: hexapodContext.connected ? "Disconnect" : "Connect"
                        highlighted: !hexapodContext.connected
                        Material.accent: hexapodContext.connected ? Material.Red : Material.Green
                        Layout.fillWidth: true
                        
                        onClicked: {
                            if (hexapodContext.connected) {
                                hexapodContext.disconnect();
                            } else {
                                // Save settings
                                connectionSettings.hostname = hostnameField.text;
                                connectionSettings.port = portSpinBox.value;
                                
                                // Connect
                                hexapodContext.connectToHost(hostnameField.text, portSpinBox.value);
                            }
                        }
                    }
                }
            }
        }
        
        GroupBox {
            title: "Log Messages"
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            ScrollView {
                id: scrollView
                anchors.fill: parent
                clip: true
                
                TextArea {
                    id: logTextArea
                    readOnly: true
                    wrapMode: TextEdit.Wrap
                    selectByMouse: true
                    
                    Connections {
                        target: hexapodContext
                        function onMessageLogged(message) {
                            logTextArea.append(message);
                            // Auto-scroll to bottom
                            scrollView.ScrollBar.vertical.position = 1.0;
                        }
                    }
                }
            }
        }
    }
}
