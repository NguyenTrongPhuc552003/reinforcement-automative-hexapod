import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

ApplicationWindow {
    id: root
    visible: true
    width: 1024
    height: 768
    title: "Hexapod Controller"
    
    Material.theme: Material.Dark
    Material.accent: Material.Blue
    Material.primary: Material.BlueGrey
    
    // App state
    property bool darkTheme: true
    property string currentTab: "dashboard"
    
    // Handle key presses for movement controls
    focus: true
    Keys.onPressed: function(event) {
        if (event.key === Qt.Key_W) {
            hexapodContext.moveForward(hexapodContext.speed);
            event.accepted = true;
        } else if (event.key === Qt.Key_S) {
            hexapodContext.moveBackward(hexapodContext.speed);
            event.accepted = true;
        } else if (event.key === Qt.Key_A) {
            hexapodContext.turnLeft(hexapodContext.speed);
            event.accepted = true;
        } else if (event.key === Qt.Key_D) {
            hexapodContext.turnRight(hexapodContext.speed);
            event.accepted = true;
        } else if (event.key === Qt.Key_Space) {
            hexapodContext.stop();
            event.accepted = true;
        }
    }
    
    Keys.onReleased: function(event) {
        if (event.key === Qt.Key_W || event.key === Qt.Key_S || 
            event.key === Qt.Key_A || event.key === Qt.Key_D) {
            hexapodContext.stop();
            event.accepted = true;
        }
    }
    
    header: ToolBar {
        Material.foreground: "white"
        
        RowLayout {
            anchors.fill: parent
            
            ToolButton {
                text: qsTr("≡")
                font.pixelSize: 24
                onClicked: drawer.open()
            }
            
            Label {
                text: "Hexapod Controller"
                font.pixelSize: 20
                elide: Label.ElideRight
                horizontalAlignment: Qt.AlignHCenter
                verticalAlignment: Qt.AlignVCenter
                Layout.fillWidth: true
            }
            
            ToolButton {
                text: "⚙"
                font.pixelSize: 20
                onClicked: settingsDialog.open()
            }
        }
    }
    
    Drawer {
        id: drawer
        width: Math.min(root.width * 0.7, 300)
        height: root.height
        
        Column {
            anchors.fill: parent
            
            ItemDelegate {
                text: "Dashboard"
                width: parent.width
                highlighted: currentTab === "dashboard"
                onClicked: {
                    currentTab = "dashboard"
                    stackView.replace("qrc:/qml/Dashboard.qml")
                    drawer.close()
                }
            }
            
            ItemDelegate {
                text: "Control Panel"
                width: parent.width
                highlighted: currentTab === "control"
                onClicked: {
                    currentTab = "control"
                    stackView.replace("qrc:/qml/ControlPanel.qml")
                    drawer.close()
                }
            }
            
            ItemDelegate {
                text: "Connection"
                width: parent.width
                highlighted: currentTab === "connection"
                onClicked: {
                    currentTab = "connection"
                    stackView.replace("qrc:/qml/ConnectionPanel.qml")
                    drawer.close()
                }
            }
            
            ItemDelegate {
                text: "Switch to Classic UI"
                width: parent.width
                onClicked: {
                    switchUIDialog.open()
                    drawer.close()
                }
            }
        }
    }
    
    Dialog {
        id: settingsDialog
        title: "Settings"
        standardButtons: Dialog.Ok | Dialog.Cancel
        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        width: Math.min(parent.width - 50, 400)
        
        ColumnLayout {
            width: parent.width
            
            Label {
                text: "Theme"
                font.bold: true
            }
            
            RadioButton {
                text: "Dark Theme"
                checked: darkTheme
                onCheckedChanged: {
                    if (checked) {
                        darkTheme = true
                        Material.theme = Material.Dark
                    }
                }
            }
            
            RadioButton {
                text: "Light Theme"
                checked: !darkTheme
                onCheckedChanged: {
                    if (checked) {
                        darkTheme = false
                        Material.theme = Material.Light
                    }
                }
            }
            
            CheckBox {
                text: "Balance Enabled"
                checked: hexapodContext.balanceEnabled
                onCheckedChanged: hexapodContext.balanceEnabled = checked
            }
        }
    }
    
    Dialog {
        id: switchUIDialog
        title: "Switch Interface?"
        standardButtons: Dialog.Yes | Dialog.No
        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        
        Label {
            width: parent.width
            wrapMode: Text.WordWrap
            text: "Are you sure you want to switch to the classic interface? The application will restart."
        }
        
        onAccepted: {
            // Save preference to settings
            let settings = Qt.createQmlObject('import Qt.labs.settings 1.0; Settings {}', root);
            settings.setValue("ui/use_qml_interface", false);
            
            // Restart the application
            Qt.quit();
        }
    }
    
    StackView {
        id: stackView
        anchors.fill: parent
        initialItem: "qrc:/qml/Dashboard.qml"
    }
    
    footer: StatusBar {
        id: statusBar
        connected: hexapodContext.connected
        statusMessage: hexapodContext.statusMessage
    }
    
    Component.onCompleted: {
        if (!hexapodContext.connected) {
            // Show connection panel if not connected
            currentTab = "connection"
            stackView.replace("qrc:/qml/ConnectionPanel.qml")
        }
    }
}
