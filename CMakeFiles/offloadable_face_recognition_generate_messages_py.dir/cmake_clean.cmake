FILE(REMOVE_RECURSE
  "CMakeFiles/offloadable_face_recognition_generate_messages_py"
  "devel/lib/python2.7/dist-packages/offloadable_face_recognition/msg/_FaceBox.py"
  "devel/lib/python2.7/dist-packages/offloadable_face_recognition/msg/_FeatureCoordinates.py"
  "devel/lib/python2.7/dist-packages/offloadable_face_recognition/srv/_PruneFeatures.py"
  "devel/lib/python2.7/dist-packages/offloadable_face_recognition/srv/_AddFeatures.py"
  "devel/lib/python2.7/dist-packages/offloadable_face_recognition/msg/__init__.py"
  "devel/lib/python2.7/dist-packages/offloadable_face_recognition/srv/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/offloadable_face_recognition_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
