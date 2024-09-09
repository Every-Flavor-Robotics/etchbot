import React, { useState } from "react";
import { Box, Heading, Input, Button, FormControl, FormLabel } from "@chakra-ui/react";
import axios from "axios";

interface CameraManagerProps {
    etchbotName: string;
}

// const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:5010';
const API_URL = 'http://localhost:5010';

const CameraManager: React.FC<CameraManagerProps> = ({ etchbotName }) => {
    const [serviceUrl, setServiceUrl] = useState("");
    const [cameraIndex, setCameraIndex] = useState(0);
    const [message, setMessage] = useState("");

    const handleConnectCamera = async () => {
        try {
            const response = await axios.post(`${API_URL}/etchbot/${etchbotName}/connect_camera`, {
                service_url: serviceUrl,
                camera_index: cameraIndex,
            });

            if (response.status === 200) {
                setMessage("Camera connected successfully.");
            } else {
                setMessage("Failed to connect camera.");
            }
        } catch (error) {
            console.error("Error connecting camera:", error);
            setMessage("Failed to connect camera.");
        }
    };

    return (
        <Box>
            <Heading as="h2" size="md" mb={4}>
                Connect Camera
            </Heading>
            <FormControl mb={4}>
                <FormLabel>Service URL</FormLabel>
                <Input
                    type="text"
                    value={serviceUrl}
                    onChange={(e) => setServiceUrl(e.target.value)}
                    placeholder="Enter the camera service URL"
                />
            </FormControl>
            <FormControl mb={4}>
                <FormLabel>Camera Index</FormLabel>
                <Input
                    type="number"
                    value={cameraIndex}
                    onChange={(e) => setCameraIndex(parseInt(e.target.value, 10))}
                    placeholder="Enter the camera index"
                />
            </FormControl>
            <Button colorScheme="teal" onClick={handleConnectCamera}>
                Connect Camera
            </Button>
            {message && (
                <Box mt={4} color={message.includes("successfully") ? "green" : "red"}>
                    {message}
                </Box>
            )}
        </Box>
    );
};

export default CameraManager;