import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ToolBar {
    property bool connected: false
    property string statusMessage: "Disconnected"
    
    height: 30
    
    background: Rectangle {
        color: connected ? "#4CAF50" : "#F44336"
    }
    
    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 10
        anchors.rightMargin: 10
        
        Label {
            text: statusMessage
            color: "white"
            font.bold: true
            Layout.fillWidth: true
        }
        
        Label {
            text: connected ? "●" : "○"
            color: "white"
            font.bold: true
            font.pixelSize: 16
        }
    }
}
