Put generated data files here.

Expected files:

- `kerbin_map.png`               (generated basemap image)
- `ifc_beacons.csv`              (from kOS exporter)
- `ifc_plates.csv`               (from kOS exporter)
- optional `generated/` folder   (raw NumPy sampled grids and metadata)

CSV schemas:

`ifc_beacons.csv`
- `id,type,lat,lon,alt_asl,name,runway`

`ifc_plates.csv`
- `plate_id,sequence,beacon_id,vapp`
