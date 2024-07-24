# Umgebung für Testzwecke starten:

- Starte Server in Terminal
    - Docker starten
    - Befehl ausführen:
    ```
    ros2 run pkg_pack_node pack_server
    ```
- 2. Terminal öffnen
    - Mit Docker verbinden: 
    ```
    docker exec -it #name# bash
    ```
    - Service call machen:

    ```
    ros2 service call pack_planning aip_packing_planning_interfaces/srv/PackSequence '{}'
    ```

Für die Simulation muss in der Datei "PackAlgorithm_Server.py" die Zeile 
```
items = ["Box_Gluehlampe", "Box_Wischblatt", "Keilriemen_gross"]
```
aktiviert werden. Hier können Objekte eingetragen werden, welche simuliert werden sollen. Die Zeile
```
items = request.objects_to_pick
```
muss für die Simulation deaktiviert werden.






ros2 service call solution_image_transfer aip_packing_planning_interfaces/srv/SolutionImage '{}'
python3 Solution_image_transfer_server.py 
