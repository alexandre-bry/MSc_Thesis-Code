# TODO

- Roofprints:
  - Try in 3D
  - Try to use the distance to the current outline as an indication as well
  - Try to extract the length of the matched edges as well instead of simply finding infinite lines
  - Try regularization on the buildings (see [this CGAL package](https://doc.cgal.org/latest/Shape_regularization/index.html) or [this paper](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=CLbvVRQAAAAJ&citation_for_view=CLbvVRQAAAAJ:Y0pCki6q_DkC) by [this researcher](https://scholar.google.com/citations?view_op=view_citation&hl=en&user=7XqwgZoAAAAJ&citation_for_view=7XqwgZoAAAAJ:_Qo2XoVZTnwC)):
    - Regularization of edges independently of the data could make results worse?
- Pipeline:
  - Script to download tiles automatically. Tiles index is available [on Zenodo](https://zenodo.org/records/13793544), and there is a [script there](https://github.com/cusicand/lidarhd_ign_downloader)
  - Script to extract and merge LiDAR flight axes
  - Integrate the script to compute the scanning vehicle position
  - Script to merge all the flight axes for a given area
