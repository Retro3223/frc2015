if ($args[0] -eq "kiwi") {
    echo "deploying kiwi drive code..."
    py kiwidrive\robot.py deploy --skip-tests
}
else if($args[0] -eq "tire") {
    echo "deploying tire drive code..."
    py tiredrive\robot.py deploy --skip-tests
}
else {
    echo "I don't understand that command!"
}