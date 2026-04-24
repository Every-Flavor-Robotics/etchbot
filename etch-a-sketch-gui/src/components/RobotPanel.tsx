import React from "react";
import { Grid, GridItem } from "@chakra-ui/react";
import NewDrawingCard from "./NewDrawingCard";
import RobotCard from "./RobotCard";
import QueueCard from "./QueueCard";
import CompletedCard from "./CompletedCard";

interface RobotPanelProps { etchbotName: string; }

const RobotPanel: React.FC<RobotPanelProps> = ({ etchbotName }) => (
  <Grid
    templateColumns={{ base: "1fr", md: "1fr 1fr" }}
    gap="14px"
    p="16px"
    alignItems="start"
  >
    {/* Left: submit input */}
    <GridItem display="flex" flexDirection="column">
      <NewDrawingCard etchbotName={etchbotName} />
    </GridItem>

    {/* Right: status → queue → completed */}
    <GridItem display="flex" flexDirection="column" gap="12px">
      <RobotCard etchbotName={etchbotName} />
      <QueueCard etchbotName={etchbotName} />
      <CompletedCard etchbotName={etchbotName} />
    </GridItem>
  </Grid>
);

export default RobotPanel;
