import React, { useState, ChangeEvent, FormEvent, useEffect } from "react";
import {
    Box,
    Button,
    Input,
    Progress,
    Alert,
    AlertIcon,
    AlertTitle,
    AlertDescription,
    CloseButton,
    Heading,
    Select,
    NumberInput,
    NumberInputField,
    FormControl,
    FormLabel,
} from "@chakra-ui/react";
import axios from "axios";

interface UploadFormProps {
    etchbotName: string;
}

const API_URL = import.meta.env.VITE_API_URL || "http://localhost:5010";

const UploadForm: React.FC<UploadFormProps> = ({ etchbotName }) => {
    // Initialize state from localStorage or defaults
    const [pipeline, setPipeline] = useState<string>(
        () => localStorage.getItem("pipeline") || "potrace"
    );
    const [framerate, setFramerate] = useState<number>(
        () => parseInt(localStorage.getItem("framerate") || "8")
    );
    const [file, setFile] = useState<File | null>(null);
    const [loading, setLoading] = useState(false);
    const [success, setSuccess] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const handleFileChange = (e: ChangeEvent<HTMLInputElement>) => {
        if (e.target.files) {
            setFile(e.target.files[0]);
        }
    };

    const handleSubmit = async (e: FormEvent) => {
        e.preventDefault();
        if (!file) return;

        const formData = new FormData();
        // Replace spaces in the file name with underscores
        const newFileName = file.name.replace(/ /g, "_");
        formData.append("file", file, newFileName);
        formData.append("pipeline", pipeline); // Append pipeline
        formData.append("framerate", framerate.toString()); // Append framerate

        setLoading(true);
        setSuccess(false);
        setError(null);

        try {
            const response = await axios.post(
                `${API_URL}/upload/${etchbotName}`,
                formData,
                {
                    headers: {
                        "Content-Type": "multipart/form-data",
                    },
                }
            );
            console.log("Success:", response.data);
            setSuccess(true);
        } catch (err) {
            console.error("Error:", err);
            setError("Failed to upload file. Please try again.");
        } finally {
            setLoading(false);
            // Optionally reset the file input after upload
            setFile(null);
        }
    };

    // Update localStorage when pipeline changes
    useEffect(() => {
        localStorage.setItem("pipeline", pipeline);
    }, [pipeline]);

    // Update localStorage when framerate changes
    useEffect(() => {
        localStorage.setItem("framerate", framerate.toString());
    }, [framerate]);

    useEffect(() => {
        if (success || error) {
            const timer = setTimeout(() => {
                setSuccess(false);
                setError(null);
            }, 3000); // Notifications disappear after 3 seconds

            return () => clearTimeout(timer);
        }
    }, [success, error]);

    return (
        <Box>
            <Heading as="h3" size="md" mb={4}>
                Upload Drawing
            </Heading>
            <form onSubmit={handleSubmit}>
                <Input
                    type="file"
                    onChange={handleFileChange}
                    mb={4}
                    value={file ? undefined : ""}
                />

                {/* Pipeline Selection with Label */}
                <FormControl mb={4}>
                    <FormLabel>Pipeline</FormLabel>
                    <Select
                        value={pipeline}
                        onChange={(e) => setPipeline(e.target.value)}
                    >
                        <option value="potrace">Potrace</option>
                        <option value="vtracer">Vtracer</option>
                    </Select>
                </FormControl>

                {/* Framerate Input with Label */}
                <FormControl mb={4}>
                    <FormLabel>Framerate</FormLabel>
                    <NumberInput
                        value={framerate}
                        onChange={(valueString) =>
                            setFramerate(parseInt(valueString) || 8)
                        }
                        min={1}
                        max={60}
                    >
                        <NumberInputField placeholder="Framerate" />
                    </NumberInput>
                </FormControl>

                <Button type="submit" colorScheme="teal" isDisabled={loading}>
                    Upload
                </Button>
            </form>

            {loading && <Progress size="xs" isIndeterminate />}

            {success && (
                <Alert status="success" mt={4}>
                    <AlertIcon />
                    <AlertTitle mr={2}>Upload successful!</AlertTitle>
                    <CloseButton
                        position="absolute"
                        right="8px"
                        top="8px"
                        onClick={() => setSuccess(false)}
                    />
                </Alert>
            )}

            {error && (
                <Alert status="error" mt={4}>
                    <AlertIcon />
                    <AlertTitle mr={2}>Upload failed!</AlertTitle>
                    <AlertDescription>{error}</AlertDescription>
                    <CloseButton
                        position="absolute"
                        right="8px"
                        top="8px"
                        onClick={() => setError(null)}
                    />
                </Alert>
            )}
        </Box>
    );
};

export default UploadForm;