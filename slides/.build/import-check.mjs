import {PresentationFile,FileBlob} from '@oai/artifact-tool';
const p=await PresentationFile.importPptx(await FileBlob.load('/home/duong-12a/fr3_hungros2/slides/.build/candidate.pptx'));console.log('Imported slides:',p.slides.items.length);
