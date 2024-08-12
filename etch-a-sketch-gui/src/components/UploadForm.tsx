import React, { useState, ChangeEvent, FormEvent } from "react";
import { Box, Button, Input, Progress, Alert, AlertIcon, AlertTitle, AlertDescription, CloseButton } from "@chakra-ui/react";
import axios from "axios";

const UploadForm: React.FC = () => {
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
        formData.append("file", file); // Ensure the key matches what the backend expects

        setLoading(true);
        setSuccess(false);
        setError(null);

        try {
            const response = await axios.post("http://localhost:5010/upload", formData, {
                headers: {
                    "Content-Type": "multipart/form-data",
                },
            });
            console.log("Success:", response.data);
            setSuccess(true);
        } catch (err) {
            console.error("Error:", err);
            setError("Failed to upload file. Please try again.");
        } finally {
            setLoading(false);
        }
    };

    return (
        <Box>
            <form onSubmit={handleSubmit}>
                <Input type="file" onChange={handleFileChange} mb={4} />
                <Button type="submit" colorScheme="teal" isDisabled={loading}>
                    Upload
                </Button>
            </form>

            {loading && <Progress size="xs" isIndeterminate />}

            {success && (
                <Alert status="success" mt={4}>
                    <AlertIcon />
                    <AlertTitle mr={2}>Upload successful!</AlertTitle>
                    <CloseButton position="absolute" right="8px" top="8px" onClick={() => setSuccess(false)} />
                </Alert>
            )}

            {error && (
                <Alert status="error" mt={4}>
                    <AlertIcon />
                    <AlertTitle mr={2}>Upload failed!</AlertTitle>
                    <AlertDescription>{error}</AlertDescription>
                    <CloseButton position="absolute" right="8px" top="8px" onClick={() => setError(null)} />
                </Alert>
            )}
        </Box>
    );
};

export default UploadForm;