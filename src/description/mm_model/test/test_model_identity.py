from mm_model.identity import robot_signature


def test_signature_tracks_current_model_and_limits_but_not_workspace_path(tmp_path):
    first=tmp_path/'first';second=tmp_path/'second';first.mkdir();second.mkdir()
    for root in (first,second):
        (root/'robot.urdf').write_text('<robot name="rangermini_xarm6"/>')
        (root/'limits.yaml').write_text('speed: 0.35\n')
    signature=robot_signature(first/'robot.urdf',[first/'limits.yaml'])
    assert signature==robot_signature(second/'robot.urdf',[second/'limits.yaml'])
    (second/'limits.yaml').write_text('speed: 0.20\n')
    assert signature!=robot_signature(second/'robot.urdf',[second/'limits.yaml'])
    (second/'robot.urdf').write_text('<robot name="old_robot"/>')
    assert signature!=robot_signature(second/'robot.urdf',[first/'limits.yaml'])


def test_changed_collision_geometry_invalidates_the_signature(tmp_path):
    urdf=tmp_path/'robot.urdf';urdf.write_text('<robot name="rangermini_xarm6"/>')
    first=robot_signature(urdf,model_data={'mm.base_collision_points':[0.,1.,2.]})
    second=robot_signature(urdf,model_data={'mm.base_collision_points':[0.,1.,3.]})
    assert first!=second
