import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ToolBar {
    property bool connected: false
    property string statusMessage: "Disconnected"
    property bool isSimulation: false
    property int pingTime: 0
    
    height: 30
    
    background: Rectangle {
        color: connected ? (isSimulation ? "#FF9800" : "#4CAF50") : "#F44336"
        Behavior on color {
            ColorAnimation { duration: 300 }
        }
    }
    
    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 10
        anchors.rightMargin: 10
        
        Label {
            text: isSimulation && connected ? "SIMULATION MODE" : statusMessage
            color: "white"
            font.bold: true
            Layout.fillWidth: true
        }
        
        Label {
            text: pingTime > 0 ? pingTime + "ms" : ""
            color: "white"
            font.pixelSize: 12
            visible: connected && pingTime > 0
        }
        
        // Animated connection indicator
        Item {
            width: 16
            height: 16
            
            Rectangle {
                anchors.centerIn: parent
                width: 12
                height: 12
                radius: 6
                color: "white"
                opacity: connected ? 1.0 : 0.5
                
                SequentialAnimation on opacity {
                    running: !connected
                    loops: Animation.Infinite
                    NumberAnimation { to: 0.2; duration: 700 }
                    NumberAnimation { to: 0.5; duration: 700 }
                }
            }
        }
    }
}
