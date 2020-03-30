<?php
$folder_path = "sensor_logs/";
$config_path = "sensor_logs/config.txt";
$config_len = 5;

$data_raw = file_get_contents("php://input");
$data_len = strlen($data_raw);

if($data_len > 0){
    if(unpack("Cmanifest", $data_raw, 0)["manifest"] > 0){
        foreach(glob($folder_path . "*.log") as $log_path){
            echo substr($log_path, strlen($folder_path)) . "\n";
        }
    } else {
        $log_path = substr($data_raw, 1);
        if(file_exists($folder_path . $log_path)){
            echo file_get_contents($folder_path . $log_path);
        }
    }
}
?>