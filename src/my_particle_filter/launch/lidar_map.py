<launch>
  <!-- LIDAR Node -->
  <node pkg="my_particle_filter" type="lidar_map.py" name="MapData_node" output="screen" />
</launch>