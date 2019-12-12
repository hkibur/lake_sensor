<?php
$folder_path = "sensor_logs/";
$config_path = "sensor_logs/config.txt";
# ms delta (unsigned long), sample average (double)
$sample_average_error_len = 4;
$sample_average_delta_len = 8;
$sample_average_len = 8;
$mac_addr_len = 6;
$device_name_len = 64;
$config_len = 5;
$total_len = $sample_average_delta_len + $sample_average_len;

$data_raw = file_get_contents("php://input");
$data_len = strlen($data_raw);

$error_char = unpack("Cerror_char", $data_raw, 0);
$mac_addr = bin2hex(substr($data_raw, 1, 1 + $mac_addr_len));
$dev_name = bin2hex(substr($data_raw, 1 + $mac_addr_len, 1 + $mac_addr_len + $device_name_len));
$filename = $dev_name . "-" . $mac_addr . ".txt";
$data_offset = 1 + $mac_addr_len + $device_name_len;

$log_reverse_array = array();

if(($data_len - $data_offset) % $total_len == 0){
    $log_handle = fopen($folder_path + $filename, "a");
    $average_time = intval(microtime(true) * 1000);
    for($i = $data_offset; $i < $data_len; $i += $total_len){
        $unpacked_arr = unpack("Pdelta_ms/eaverage", $data_raw, $i);
        $log_string = date("Y-m-d H:i:s", intval($average_time / 1000)) . ":" . strval($average_time % 10000) . " | " . strval($unpacked_arr["average"]) . "\n";
        $average_time -= $unpacked_arr["delta_ms"];
        array_unshift($log_reverse_array, $log_string);
    }
    foreach($log_reverse_array as $log_string){
        fwrite($log_handle, $log_string);
    }
    fclose($log_handle);
    $ret_handle = fopen($config_path, "r");
    echo fread($ret_handle, $config_len);
    fclose($ret_handle);
}
?>