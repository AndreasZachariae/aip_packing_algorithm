# AIP_pack_algorithm

## General structure

1. Order list is received via topic
2. Compare objets on list with material master
3. Material master extracts neccesary information and saves these in order data
4. Packing algorithm runs and returns preliminary packplan. Container dimensions is hard coded.
5. Cylinders for each package is chosen and saved in packplan
6. Place coordinates for each package are calculated and saved in packplan
7. Unneccesary information are deleted in packplan
8. Packplan is published on topic
