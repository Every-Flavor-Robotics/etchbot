// src/App.tsx
import React from "react";
import { ChakraProvider, Box, Heading } from "@chakra-ui/react";
import UploadForm from "./components/UploadForm";

const App: React.FC = () => {
    return (
        <ChakraProvider>
            <Box p={5}>
                <Heading as="h1" mb={5}>
                    Image Upload
                </Heading>
                <UploadForm />
            </Box>
        </ChakraProvider>
    );
};

export default App;