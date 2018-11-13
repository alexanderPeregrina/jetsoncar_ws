<launch>
    <!-- Running Joy node -->
    <node respawn="true" pkg="joy"
        type="joy_node" name="jetson_joy" >
        <param name="dev" type="string" value="/dev/input/js0" />
        <param name="deadzone" value="0.12" />
    </node>

    <!-- Running CNN controller Node -->
    <node pkg="jetsoncar_rl" type="cnn_controller.py" name="cnn_control"/>

    <!-- Rosserial Arduino -->
    <node name="serial_node" pkg="rosserial_python" type="serial_node.py">
            <param name="port" value="/dev/ttyUSB0"/>
            <param name="baud" value="57600" />
    </node>
</launch>
