import QtQuick
import QtQuick.Controls

ToolTip {
    id: tip

    property bool darkTheme: true

    delay: 420
    timeout: 5000
    leftPadding: 8
    rightPadding: 8
    topPadding: 5
    bottomPadding: 5

    contentItem: Label {
        text: tip.text
        color: tip.darkTheme ? "#E5ECF5" : "#263447"
        font.pixelSize: 9
        font.weight: Font.DemiBold
    }

    background: Rectangle {
        radius: 6
        color: tip.darkTheme ? "#202A39" : "#FFFFFF"
        border.width: 1
        border.color: tip.darkTheme ? "#3A475A" : "#CBD5E1"
    }
}
