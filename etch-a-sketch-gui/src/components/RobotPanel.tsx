import React from "react";
import { Box, Heading, Grid, GridItem, Divider } from "@chakra-ui/react";
import UploadForm from "./UploadForm";
import QueueStatus from "./QueueStatus";
import CompletedStatus from "./CompletedStatus";
import CameraManager from "./CameraManager";
import RobotStatus from "./RobotStatus"; // Import the RobotStatus component

interface RobotPanelProps {
    etchbotId: string;
    etchbotName: string;
}

const RobotPanel: React.FC<RobotPanelProps> = ({ etchbotId, etchbotName }) => {
    return (
        <Box p={5}>
            <Heading as="h1" mb={5} textAlign="center">
                {etchbotName} Panel
            </Heading>
            <Grid templateColumns="repeat(2, 1fr)" gap={6}>
                <GridItem colSpan={2}>
                    <RobotStatus etchbotName={etchbotName} />
                </GridItem>
                <GridItem colSpan={2}>
                    <Divider />
                </GridItem>
                <GridItem colSpan={[2, 1]}>
                    <UploadForm etchbotName={etchbotName} />
                </GridItem>
                <GridItem colSpan={[2, 1]}>
                    <CameraManager etchbotId={etchbotId} etchbotName={etchbotName} />
                </GridItem>
                <GridItem colSpan={2}>
                    <Divider />
                </GridItem>
                <GridItem colSpan={[2, 1]}>
                    <QueueStatus etchbotName={etchbotName} />
                </GridItem>
                <GridItem colSpan={[2, 1]}>
                    <CompletedStatus etchbotName={etchbotName} />
                </GridItem>
            </Grid>
        </Box>
    );
};

export default RobotPanel;
