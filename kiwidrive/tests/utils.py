def assert_called_with_fuzzy(mock, expected_arg, places=3):
    assert mock.called
    actual_arg = mock.call_args[0][0]
    assert round(expected_arg - actual_arg, places) == 0, \
        "expected: %s ± %s, actual: %s" % (
            expected_arg, 0.1 ** places, actual_arg)
    mock.reset_mock()
