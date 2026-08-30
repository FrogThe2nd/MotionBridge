import QtQuick
import QtQuick.Controls

ScrollBar {
    id: bar

    property bool darkTheme: true

    hoverEnabled: true
    padding: 2
    minimumSize: 0.08
    implicitWidth: orientation === Qt.Vertical ? 8 : 0
    implicitHeight: orientation === Qt.Horizontal ? 8 : 0

    contentItem: Rectangle {
        implicitWidth: 4
        implicitHeight: 4
        radius: 2
        color: bar.darkTheme ? "#59677A" : "#9BA9B9"
        opacity: bar.active || bar.hovered || bar.pressed ? 0.9 : 0.48

        Behavior on opacity {
            NumberAnimation { duration: 120 }
        }
    }

    background: Item {}
}
